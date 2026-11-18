/*
 * MediaTek GCE / CMDQ mailbox model for the MT6895 machine.
 *
 * The GCE is the command engine the CMDQ driver (mtk-cmdq-drv-ext) drives:
 * Linux writes a command buffer address and length into a thread, enables the
 * thread, and waits for the thread to report completion by interrupt.  This
 * model implements
 *
 *   - the thread state machine the driver polls (warm reset self-clear,
 *     CURR_ADDR/END_ADDR, suspend/ack, per-thread IRQ status/enable, the
 *     global CURR_IRQ_STATUS clear-bit convention), and
 *   - a *minimal CMDQ instruction executor*: on thread enable the model fetches
 *     64-bit instructions from guest physical memory (the command buffer lives
 *     in guest DRAM) and applies the subset of CMDQ opcodes the vendor display
 *     driver actually emits: MASK, WRITE (with/without mask), WRITE_S/READ_S
 *     (GPR/SPR addressed writes and reads), LOGIC (GPR assign), POLL, WFE,
 *     JUMP and EOC.
 *
 * Why this is needed: the kernel's mediatek-drm (and the DRM fbdev/minui
 * path) configure OVL/RDMA/DSI entirely through CMDQ packets.  With a pure
 * state machine (previous model) the packets completed but no register was
 * ever written, so the OVL windows never changed after LK handed over and the
 * QEMU display stayed on the LK logo forever .
 *
 * Instruction encoding (little endian, 64 bits; see the vendor kernel's
 * drivers/soc/mediatek/mtk-cmdq-helper.c and upstream
 * include/linux/mailbox/mtk-cmdq-mailbox.h):
 *
 *   bits  0..31  argument A: value / polled value / mask / jump target
 *   bits 32..47  argument B: register offset / event / destination GPR
 *   bits 48..55  subsystem id, or GPR flags (sop:5, arg_c_t, src_t, dst_t)
 *   bits 56..63  opcode (CMDQ_CODE_*)
 *
 *   WRITE:  { sub, offset, value }          -> *(subsys[sub] + offset) = value
 *   MASK:   mask = arg_a (~mask), or GPR assign when dst_t (bit 55) is set
 *   WRITE with offset bit0 set: look up the current value and merge
 *           new = (old & mask) | (value & ~mask)
 *   WRITE_S: PA = (GPR[sop] << 16) | offset, value from GPR/imm
 *   READ_S:  GPR[dst] = *(GPR[sop] << 16 | addr_low)
 *   POLL:   spin until (reg & ~mask) == (value & ~mask) - the model never
 *           blocks (a mismatch is logged and execution continues so a wrong
 *           assumption can never hang the guest)
 *   WFE:    wait for a CMDQ event, or set/clear one; a wait on an event a
 *           modelled block raises parks the thread until that event fires
 *   JUMP:   absolute (arg_b == 1) or relative; EOC ends a task (IRQ if bit0)
 *
 * The subsystem id -> base address table comes from the vendor device tree
 * (gce-subsys in /soc/dispsys_config@14000000:
 *  id 1 -> 0x14000000, id 2 -> 0x14010000, id 3 -> 0x14020000).
 * MTK_GCE_SUBSYS="id:base,id:base" overrides it.
 *
 * Environment:
 *   MTK_GCE_EXEC=off|dump|on  (default on; "off" is the old behaviour:
 *                             complete the thread without fetching anything,
 *                             "dump" fetches and logs but writes nothing)
 *   MTK_GCE_SUBSYS=...        subsystem id -> base map override
 *   MTK_GCELOG=1              trace thread runs, opcode histogram and (bounded)
 *                             instruction stream
 *   MTK_NO_GCE=1              machine option in hw/arm/mt6895.c: no GCE device
 *                             at all (plain syscon window, pre-1,293 behaviour)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qom/object.h"
#include "system/address-spaces.h"
#include "qemu/timer.h"
#include "system/memory.h"
#include "hw/misc/mtk_iommu.h"
#include "hw/misc/mtk_gce.h"

#define TYPE_MTK_GCE "mtk-gce"
OBJECT_DECLARE_SIMPLE_TYPE(MtkGceState, MTK_GCE)

#define MTK_GCE_SIZE            0x4000
#define MTK_GCE_NUM_THREADS     32
#define MTK_GCE_THR_BASE        0x100
#define MTK_GCE_THR_STRIDE      0x80
#define MTK_GCE_IRQ_MASK        0x00ffffffu
#define MTK_GCE_SPR_NUM         32
#define MTK_GCE_MAX_STEPS       65536

/* Global registers. */
#define GCE_CURR_IRQ_STATUS     0x010
#define GCE_THR_SLOT_CYCLES     0x030
#define GCE_SYNC_TOKEN_UPDATE   0x068

/* Per-thread registers, relative to the thread base. */
#define THR_WARM_RESET          0x00
#define   THR_DO_WARM_RESET     (1u << 0)
#define THR_ENABLE_TASK         0x04
#define   THR_ENABLED           (1u << 0)
#define THR_SUSPEND_TASK        0x08
#define THR_CURR_STATUS         0x0c
#define   THR_STATUS_SUSPENDED  (1u << 1)
#define THR_IRQ_STATUS          0x10
#define   THR_IRQ_DONE          (1u << 0)
#define   THR_IRQ_ERROR         0x12u
#define   THR_IRQ_EN            (THR_IRQ_ERROR | THR_IRQ_DONE)
#define THR_IRQ_ENABLE          0x14
#define THR_CURR_ADDR           0x20
#define THR_END_ADDR            0x24
#define THR_WAIT_TOKEN          0x30
#define THR_PRIORITY            0x40

/* CMDQ opcodes (vendor/upstream mtk-cmdq-helper.c). */
#define CMDQ_CODE_MASK          0x02
#define CMDQ_CODE_WRITE         0x04
#define CMDQ_CODE_POLL          0x08
#define CMDQ_CODE_JUMP          0x10
#define CMDQ_CODE_WFE           0x20
#define CMDQ_CODE_EOC           0x40
#define CMDQ_CODE_READ_S        0x80
#define CMDQ_CODE_WRITE_S       0x90
#define CMDQ_CODE_WRITE_S_MASK  0x91
#define CMDQ_CODE_LOGIC         0xa0
#define CMDQ_CODE_JUMPC         0xb0

/*
 * Wait-for-event (WFE) option bits, as encoded by the vendor CMDQ helpers
 * (cmdq_pkt_wfe / cmdq_pkt_set_event / cmdq_pkt_clear_event):
 *
 *   cmdq_pkt_wfe(ev, clear)  value = CMDQ_WFE_WAIT | CMDQ_WFE_WAIT_VALUE
 *                                   | (clear ? CMDQ_WFE_UPDATE : 0)
 *                            -> 0x80008001: wait for the event, then clear it
 *   cmdq_pkt_set_event(ev)   value = CMDQ_WFE_UPDATE | CMDQ_WFE_UPDATE_VALUE
 *                            -> 0x80010000: set the event
 *   cmdq_pkt_clear_event(ev) value = CMDQ_WFE_UPDATE
 *                            -> 0x80000000: clear the event
 * Both 0x80008001 and 0x80000000 appear in the recorded display packet trace,
 * so these bit assignments are what the guest actually emits.  The event id
 * travels in arg_b (instruction bits 32..47).
 */
#define CMDQ_WFE_UPDATE         (1u << 31)
#define CMDQ_WFE_UPDATE_VALUE   (1u << 16)
#define CMDQ_WFE_WAIT           (1u << 15)
#define CMDQ_WFE_WAIT_VALUE     (1u << 0)
#define MTK_GCE_NUM_EVENTS      1024    /* CMDQ_EVENT_MAX is 0x3FF */

/*
 * CMDQ_CODE_EOC carries the IRQ flag in arg_c (instruction bits 0..15): the
 * vendor helper's cmdq_pkt_eoc() writes CMDQ_EOC_IRQ_EN = BIT(0) there and
 * puts the "do not increment the exec count" flag in s_op (bits 48..52), so
 * only arg_c bit0 decides whether the done IRQ is raised.
 */
#define CMDQ_EOC_IRQ_EN         (1u << 0)
/* One CMDQ instruction is 64 bits (vendor CMDQ_INST_SIZE). */
#define CMDQ_INST_SIZE          8
/* Raw words buffered before the first DISP-touching write of a packet. */
#define MTK_GCE_RAW_RING        64

typedef struct MtkGceThread {
    uint32_t curr_addr;
    uint32_t end_addr;
    uint32_t enable;
    uint32_t suspended;
    uint32_t irq_status;
    uint32_t irq_enable;
    uint32_t priority;
    uint32_t spr[MTK_GCE_SPR_NUM];
    uint32_t mask;
    uint32_t mask_valid;
    uint32_t n_eoc;                 /* EOC instructions that raised the IRQ */
    uint32_t n_jumps;               /* JUMP instructions executed           */
} MtkGceThread;

struct MtkGceState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[MTK_GCE_SIZE / 4];
    MtkGceThread thr[MTK_GCE_NUM_THREADS];
    uint32_t pending;
    uint32_t n_started;
    bool log;
    bool exec;              /* execute instructions (default) */
    bool exec_write;        /* really write registers ("dump" mode: false) */
    bool instlog;           /* dump instruction stream */
    uint32_t subsys[256];   /* subsys id -> base address */
    uint64_t n_inst;        /* instructions executed */
    uint32_t opcnt[256];    /* opcode histogram */
    uint32_t n_unknown;
    uint32_t n_badfetch;
    uint32_t n_pollfail;
    uint32_t n_logged;
    uint32_t n_events;
    uint64_t n_jumps;           /* JUMP instructions executed (all threads)  */
    uint64_t n_eoc;             /* EOC instructions that raised the IRQ      */
    uint32_t n_steplimit;       /* exec runs stopped by the instruction cap  */
    uint32_t n_parked_log;      /* bounded "packet parked" trace lines       */
    bool raw_log;               /* MTK_GCE_RAW=1: raw words of DISP packets  */
    bool reg_log;               /* MTK_GCE_REGLOG=1: thread register traffic */
    uint32_t n_reg_logged;      /* bounded register trace lines              */
    /*
     * CMDQ sync events.  Only events a modelled peripheral block actually
     * raises (see mtk_gce_event_mark_generated) may block a WFE; for every
     * other event the wait keeps the legacy "count and skip" behaviour, so a
     * missing event model can never hang a guest commit packet.
     */
    uint32_t event_bits[MTK_GCE_NUM_EVENTS / 32];
    uint32_t event_generated[MTK_GCE_NUM_EVENTS / 32];
    uint8_t event_skip_logged[MTK_GCE_NUM_EVENTS / 8];
    bool wfe_log;               /* log WFE block/resume and skipped ids   */
    bool wfe_waiting[MTK_GCE_NUM_THREADS];
    uint32_t wfe_event[MTK_GCE_NUM_THREADS];
    bool wfe_clear[MTK_GCE_NUM_THREADS];
    uint32_t n_wfe_blocked;
    uint32_t n_wfe_resumed;
    uint32_t n_wfe_skipped;
    uint32_t n_wfe_logs;
    /*
     * GCE address space != guest physical address space (mminfra): the driver
     * programs CURR_ADDR/END_ADDR with cmdq_convert_gce_addr(pa) =
     * (pa + mminfra_offset) >> shift, so a fetched instruction address has to
     * be translated back before address_space_read().  The offset is either
     * forced by MTK_GCE_MMINFRA or calibrated from the first packet (see
     * mtk_gce_calibrate()).
     */
    uint32_t xlate_off;      /* mminfra_offset (bytes), see mtk_gce_xlate_to_pa */
    uint32_t xlate_shift;    /* shift_pa */
    bool xlate_post;         /* gce = (dma + off) >> shift (vendor mt6895) */
    bool xlate_done;
    bool xlate_fixed;
};

/*
 * Display-write epoch shared with hw/display/mtk_disp.c: bumped for every
 * register write that lands in the display windows.  mtk_disp uses it to take
 * a frame dump after each CMDQ commit even when the composed picture happens
 * to be identical to the previous dump (a plain CRC-change trigger misses
 * commits that only move a layer address).
 */
uint32_t mtk_gce_disp_epoch;

static void mtk_gce_update_irq(MtkGceState *s)
{
    qemu_set_irq(s->irq, s->pending != 0);
}

/*
 * The GCE-level CURR_IRQ_STATUS register is the OR of the per-thread IRQ
 * status bits that are enabled in THR_IRQ_ENABLE (bit 0 = done, bits 1/4 =
 * error); it is not a separately latched register the driver writes.  Recompute
 * it whenever a thread's status or enable mask changes, exactly as the
 * hardware wires the two registers together.
 */
static void mtk_gce_refresh_pending(MtkGceState *s)
{
    uint32_t pending = 0;
    unsigned idx;

    for (idx = 0; idx < MTK_GCE_NUM_THREADS; idx++) {
        MtkGceThread *t = &s->thr[idx];

        if (t->irq_status & t->irq_enable & THR_IRQ_EN) {
            pending |= 1u << idx;
        }
    }
    if (pending != s->pending) {
        s->pending = pending;
        mtk_gce_update_irq(s);
    }
}

/* EOC with CMDQ_EOC_IRQ_EN: latch THR_IRQ_STATUS.DONE and raise the SPI. */
static void mtk_gce_eoc_irq(MtkGceState *s, unsigned idx)
{
    MtkGceThread *t = &s->thr[idx];

    t->irq_status |= THR_IRQ_DONE;
    mtk_gce_refresh_pending(s);
}

/* Invalid fetch / misaligned jump: latch the error status bits like the GCE. */
static void mtk_gce_error_irq(MtkGceState *s, unsigned idx)
{
    MtkGceThread *t = &s->thr[idx];

    t->irq_status |= THR_IRQ_ERROR;
    mtk_gce_refresh_pending(s);
}

/*
 * Display windows a CMDQ packet can touch: DISP0 (0x1400xxxx, mmsys/Ovl0) and
 * DISP1 (0x1440xxxx, the second display path).  MTK_GCE_RAW dumps the raw
 * instruction words of packets that write one of them so the model can be
 * compared word for word with the driver's cmdq_log output.
 */
static bool mtk_gce_pa_is_disp(uint32_t pa)
{
    return (pa >= 0x14000000u && pa < 0x14100000u) ||
           (pa >= 0x14400000u && pa < 0x14500000u);
}

/* Flush the buffered prefix of a packet that has just written DISP. */
static void mtk_gce_raw_disp(unsigned idx, const uint64_t *words,
                             const uint32_t *pcs, unsigned n, bool *active)
{
    unsigned i;
    unsigned first;

    if (*active) {
        return;
    }
    *active = true;
    first = n > MTK_GCE_RAW_RING ? n - MTK_GCE_RAW_RING : 0;
    for (i = first; i < n; i++) {
        fprintf(stderr, "MTK-GCE-RAW thr%u pc=%08x inst=%016" PRIx64 "\n",
                idx, pcs[i & (MTK_GCE_RAW_RING - 1)],
                words[i & (MTK_GCE_RAW_RING - 1)]);
    }
}

/* Guest time in seconds, for the event trace messages. */
static double mtk_gce_guest_seconds(void)
{
    return (double)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 1e9;
}

static bool mtk_gce_event_isset(MtkGceState *s, unsigned event)
{
    if (event >= MTK_GCE_NUM_EVENTS) {
        return false;
    }
    return (s->event_bits[event >> 5] >> (event & 31)) & 1u;
}

static void mtk_gce_event_put(MtkGceState *s, unsigned event, bool set)
{
    uint32_t *word;

    if (event >= MTK_GCE_NUM_EVENTS) {
        return;
    }
    word = &s->event_bits[event >> 5];
    if (set) {
        *word |= 1u << (event & 31);
    } else {
        *word &= ~(1u << (event & 31));
    }
}

static bool mtk_gce_event_is_generated(MtkGceState *s, unsigned event)
{
    if (event >= MTK_GCE_NUM_EVENTS) {
        return false;
    }
    return (s->event_generated[event >> 5] >> (event & 31)) & 1u;
}

static void mtk_gce_event_mark(MtkGceState *s, unsigned event)
{
    if (event < MTK_GCE_NUM_EVENTS) {
        s->event_generated[event >> 5] |= 1u << (event & 31);
    }
}

static bool mtk_gce_thread_lookup(hwaddr addr, unsigned *idx, unsigned *reg)
{
    hwaddr rel;

    if (addr < MTK_GCE_THR_BASE) {
        return false;
    }
    rel = addr - MTK_GCE_THR_BASE;
    if (rel >= MTK_GCE_NUM_THREADS * MTK_GCE_THR_STRIDE) {
        return false;
    }
    *idx = rel / MTK_GCE_THR_STRIDE;
    *reg = rel % MTK_GCE_THR_STRIDE;
    return true;
}

static void mtk_gce_dump_ops(MtkGceState *s)
{
    fprintf(stderr,
            "MTK-GCE ops: MASK=%u WRITE=%u POLL=%u JUMP=%u WFE=%u EOC=%u "
            "WR_S=%u RD_S=%u LOGIC=%u UNK=%u steps=%" PRIu64
            " unknown=%u badfetch=%u pollfail=%u events=%u"
            " jumps=%" PRIu64 " eoc=%" PRIu64 "\n",
            s->opcnt[CMDQ_CODE_MASK], s->opcnt[CMDQ_CODE_WRITE],
            s->opcnt[CMDQ_CODE_POLL], s->opcnt[CMDQ_CODE_JUMP],
            s->opcnt[CMDQ_CODE_WFE], s->opcnt[CMDQ_CODE_EOC],
            s->opcnt[CMDQ_CODE_WRITE_S] + s->opcnt[CMDQ_CODE_WRITE_S_MASK],
            s->opcnt[CMDQ_CODE_READ_S], s->opcnt[CMDQ_CODE_LOGIC],
            s->opcnt[0], s->n_inst, s->n_unknown, s->n_badfetch,
            s->n_pollfail, s->n_events, s->n_jumps, s->n_eoc);
}

static uint32_t mtk_gce_read32(MtkGceState *s, uint32_t pa)
{
    uint32_t v = 0xffffffffu;

    address_space_read(&address_space_memory, pa, MEMTXATTRS_UNSPECIFIED,
                       &v, sizeof(v));
    return v;
}

static void mtk_gce_write32(MtkGceState *s, uint32_t pa, uint32_t v)
{
    address_space_write(&address_space_memory, pa, MEMTXATTRS_UNSPECIFIED,
                        &v, sizeof(v));
}

static uint32_t mtk_gce_subsys_addr(MtkGceState *s, unsigned sub, unsigned off,
                                    bool *known)
{
    uint32_t base = s->subsys[sub & 0xffu];

    if (known) {
        *known = base != 0;
    }
    if (!base) {
        return off;
    }
    return base + (off & 0xfffcu);
}

/*
 * A PA-mode command buffer addresses a register as GPR[sop](high 16 bits) |
 * offset(low 16 bits) - the vendor helper's cmdq_pkt_assign(
 * CMDQ_ADDR_HIGH(pa)) + cmdq_pkt_write_s(..., CMDQ_ADDR_LOW(pa), ...) pair.
 *
 * Some packets instead store the whole 32-bit address in the GPR.  Accept
 * both: if the "high" part already carries more than 16 bits of address use
 * it as the complete base instead of shifting it in.
 */
static uint32_t mtk_gce_gpr_addr(MtkGceThread *t, unsigned sop, unsigned off)
{
    uint32_t hi = t->spr[sop & (MTK_GCE_SPR_NUM - 1)];
    /*
     * arg_b bits[1:0] are instruction flags, not address bits: every
     * vendor display CMDQ write observed on mt6895 carries arg_b == 2
     * (mod 4) - e.g. the 0x1400205e / 0x1440206e / 0x1440208a traces are
     * really 0x1400205c / 0x1440206c / 0x14402088.  Mask the flag bits
     * before composing the physical address.
     */
    uint32_t a = off & 0xfffcu;

    if (hi & 0xffff0000u) {
        return hi + a;
    }
    return (hi << 16) | a;
}

static void mtk_gce_log_inst(MtkGceState *s, unsigned idx, uint32_t pc,
                             uint64_t raw)
{
    static const char *names[256];
    unsigned op = raw >> 56;
    uint32_t val = raw;
    unsigned sub = (raw >> 48) & 0xff;
    unsigned off = (raw >> 32) & 0xffff;
    const char *name = names[op];

    if (!name) {
        switch (op) {
        case CMDQ_CODE_MASK:         name = "MASK"; break;
        case CMDQ_CODE_WRITE:        name = "WRITE"; break;
        case CMDQ_CODE_POLL:         name = "POLL"; break;
        case CMDQ_CODE_JUMP:         name = "JUMP"; break;
        case CMDQ_CODE_WFE:          name = "WFE"; break;
        case CMDQ_CODE_EOC:          name = "EOC"; break;
        case CMDQ_CODE_READ_S:       name = "READ_S"; break;
        case CMDQ_CODE_WRITE_S:      name = "WRITE_S"; break;
        case CMDQ_CODE_WRITE_S_MASK: name = "WRITE_S_MASK"; break;
        case CMDQ_CODE_LOGIC:        name = "LOGIC"; break;
        default:                     name = "?"; break;
        }
        names[op] = name;
    }
    if (op == CMDQ_CODE_WRITE) {
        bool known;
        uint32_t pa = mtk_gce_subsys_addr(s, sub, off, &known);

        fprintf(stderr,
                "MTK-GCE-I thr%u pc=%08x inst=%016" PRIx64 " %-12s sub=%u(%#x) "
                "off=%04x val=%08x -> pa=%08x%s\n",
                idx, pc, raw, name, sub, s->subsys[sub & 0xff], off, val, pa,
                known ? "" : " (no-subsys)");
    } else if (op == CMDQ_CODE_MASK || op == CMDQ_CODE_LOGIC) {
        unsigned dst = (raw >> 32) & 0xffff;

        fprintf(stderr,
                "MTK-GCE-I thr%u pc=%08x inst=%016" PRIx64 " %-12s dst=%u val=%08x "
                "sop=%u dst_t=%u\n",
                idx, pc, raw, name, dst, val, (unsigned)((raw >> 48) & 0x1f),
                (unsigned)((raw >> 55) & 1));
    } else {
        fprintf(stderr,
                "MTK-GCE-I thr%u pc=%08x inst=%016" PRIx64 " %-12s arg_b=%08x "
                "val=%08x\n",
                idx, pc, raw, name, (uint32_t)(raw >> 32), val);
    }
}

static void mtk_gce_thread_reset(MtkGceState *s, unsigned idx)
{
    memset(&s->thr[idx], 0, sizeof(s->thr[idx]));
    s->wfe_waiting[idx] = false;
    if (s->pending & (1u << idx)) {
        s->pending &= ~(1u << idx);
        mtk_gce_update_irq(s);
    }
}

/* The modelled GCE finishes a queued buffer immediately. */
static void mtk_gce_thread_complete(MtkGceState *s, unsigned idx)
{
    MtkGceThread *t = &s->thr[idx];

    t->curr_addr = t->end_addr;
    t->irq_status |= THR_IRQ_DONE;
    mtk_gce_refresh_pending(s);
    s->n_started++;
    if (s->log && s->n_started <= 64) {
        fprintf(stderr,
                "MTK-GCE %s thread %u done pc=0x%08x end=0x%08x "
                "irq_en=0x%x insts=%" PRIu64 "\n",
                memory_region_name(&s->iomem), idx, t->curr_addr, t->end_addr,
                t->irq_enable, s->n_inst);
    }
}

/*
 * Translate a GCE-domain thread/fetch address back to the guest address the
 * CPU uses.  Two vendor conversions are possible (both print themselves as
 * "cmdq thread:N shift:S mminfra:0xM" at probe):
 *
 *   post (default; the vendor 5.10 mt6895 cmdq driver - disassembled from
 *         mtk-cmdq-drv-ext.ko, the thread-dump path does
 *             ldr  w9,  [gce_shift_bit]
 *             ldrsw x10, [gce_mminfra]
 *             lsl  x8,  x8, x9
 *             sub  x8,  x8, x10
 *         i.e. dma = (gce << shift) - mminfra, so the forward conversion is
 *         gce = (dma + mminfra) >> shift):
 *
 *   pre  (mt8183/mt8192 style, kept for the older experiments):
 *         gce = (dma >> shift) + mminfra.
 *
 * "dma" is the DMA address of the cm dq buffer: a plain physical address when
 * the GCE is not behind an IOMMU, an IOVA when it is (see the DT "iommus"
 * property of the gce node).
 */
static bool mtk_gce_xlate_to_pa(MtkGceState *s, uint32_t gce_addr, hwaddr *pa)
{
    hwaddr dma;

    if (s->xlate_post) {
        hwaddr a = (hwaddr)gce_addr << s->xlate_shift;

        if (a < s->xlate_off) {
            return false;
        }
        dma = a - s->xlate_off;
    } else {
        if (gce_addr < s->xlate_off) {
            return false;
        }
        dma = (hwaddr)(gce_addr - s->xlate_off) << s->xlate_shift;
    }

    /*
     * The command buffer is allocated with dma_alloc_coherent(): its DMA
     * address is a physical address without an IOMMU and an IOVA when the GCE
     * is behind the real M4U model (DT: gce@1e980000 iommus=<&mdp_iommu ...>).
     */
    /*
     * mtk_iommu_translate_or_identity() covers both cases uniformly: the M4U
     * page table is consulted first, an unmapped DMA address below 4 GiB that
     * is backed by guest RAM (the bootloader/LK buffers, e.g. the logo
     * framebuffer, which are plain physical addresses) is used as-is, and
     * everything else is a real fault.  IOVAs are 34-bit on this SoC, so the
     * full hwaddr must be passed - never truncated to 32 bits.
     */
    return mtk_iommu_translate_or_identity(MTK_IOMMU_DEV_MDP, dma, pa);
}

/*
 * CMDQ instruction fetch: the address the driver programs into the thread is a
 * GCE-domain address; translate it back into a guest physical address first.
 */
static bool mtk_gce_fetch(MtkGceState *s, uint32_t gce_addr, uint64_t *raw)
{
    hwaddr pa;
    MemTxResult r;

    if (!mtk_gce_xlate_to_pa(s, gce_addr, &pa)) {
        return false;
    }
    r = address_space_read(&address_space_memory, pa,
                           MEMTXATTRS_UNSPECIFIED, raw, sizeof(*raw));
    if (r != MEMTX_OK) {
        return false;
    }
    *raw = le64_to_cpu(*raw);
    return true;
}

static bool mtk_gce_op_known(uint64_t raw)
{
    switch (raw >> 56) {
    case CMDQ_CODE_MASK:
    case CMDQ_CODE_WRITE:
    case CMDQ_CODE_POLL:
    case CMDQ_CODE_JUMP:
    case CMDQ_CODE_WFE:
    case CMDQ_CODE_EOC:
    case CMDQ_CODE_READ_S:
    case CMDQ_CODE_WRITE_S:
    case CMDQ_CODE_WRITE_S_MASK:
    case CMDQ_CODE_LOGIC:
    case CMDQ_CODE_JUMPC:
        return true;
    default:
        return false;
    }
}

/*
 * Score a candidate GCE-address -> PA offset by looking at the first few
 * instructions of the packet and at the terminating instruction.  A real
 * command buffer scores high; an address that lands in image data or in
 * uninitialised memory scores low.
 */
static int mtk_gce_score(MtkGceState *s, uint32_t addr, uint32_t off,
                         uint32_t shift)
{
    uint32_t save_off = s->xlate_off;
    uint32_t save_shift = s->xlate_shift;
    int score = 0;
    int i;

    s->xlate_off = off;
    s->xlate_shift = shift;
    for (i = 0; i < 4; i++) {
        uint64_t raw;

        if (!mtk_gce_fetch(s, addr + 8 * i, &raw)) {
            score = -100;
            break;
        }
        if (mtk_gce_op_known(raw)) {
            score += 3;
        } else if (raw == 0 || raw == ~0ULL) {
            score -= 2;
        } else {
            score -= 1;
        }
    }
    if (score >= 0) {
        uint64_t raw;

        if (mtk_gce_fetch(s, (addr & ~7u), &raw) && mtk_gce_op_known(raw)) {
            score += 2;
        }
    }
    s->xlate_off = save_off;
    s->xlate_shift = save_shift;
    return score;
}

static void mtk_gce_calibrate(MtkGceState *s, uint32_t addr)
{
    static const struct {
        uint32_t off;
        uint32_t shift;
    } cands[] = {
        { 0x40000000u, 3 },     /* vendor mt6895: shift:3 mminfra:0x40000000 */
        { 0x40000000u, 0 },
        { 0x00000000u, 3 },
        { 0x00000000u, 0 },
        { 0x40000000u, 4 },
        { 0x40000000u, 16 },
        { 0x40000000u, 6 },
        { 0x80000000u, 0 },
    };
    int best = -1000;
    uint32_t best_off = 0;
    uint32_t best_shift = 0;
    size_t i;

    if (s->xlate_done) {
        return;
    }
    if (s->xlate_fixed) {
        s->xlate_done = true;
        return;
    }
    for (i = 0; i < ARRAY_SIZE(cands); i++) {
        int sc = mtk_gce_score(s, addr, cands[i].off, cands[i].shift);

        if (sc > best) {
            best = sc;
            best_off = cands[i].off;
            best_shift = cands[i].shift;
        }
    }
    s->xlate_off = best_off;
    s->xlate_shift = best_shift;
    s->xlate_done = true;
    if (s->log) {
        uint64_t w[4] = { 0, 0, 0, 0 };
        hwaddr pa = 0;
        int wi;

        if (!mtk_gce_xlate_to_pa(s, addr, &pa)) {
            pa = 0;
        }

        for (wi = 0; wi < 4; wi++) {
            mtk_gce_fetch(s, addr + 8 * wi, &w[wi]);
        }
        fprintf(stderr,
                "MTK-GCE xlate: gce=0x%08x -> pa=0x%" PRIx64
                " (mminfra=0x%x shift=%u score=%d)\n"
                "MTK-GCE xlate words: %016" PRIx64 " %016" PRIx64 " %016" PRIx64
                " %016" PRIx64 "\n", addr, (uint64_t)pa, best_off, best_shift,
                best, w[0], w[1], w[2], w[3]);
    }
}

/*
 * Execute the command buffer queued on thread @idx.
 *
 * Architectural PC arithmetic: the GCE programs CURR_ADDR/END_ADDR as
 * CMDQ_REG_SHIFT_ADDR(pa) = (pa + gce_mminfra) >> gce_shift_bit (the driver
 * prints "cmdq thread:32 shift:3 mminfra:0x40000000"), so one GCE address unit
 * is 1 << shift bytes and an instruction advances the PC by
 * CMDQ_INST_SIZE >> shift (1 unit at shift 3, 8 units at shift 0).  The
 * MTK_GCE_XLATE/MTK_GCE_SHIFT environment only selects how the PC is
 * translated back to a guest physical address in mtk_gce_fetch().
 *
 * END_ADDR is the thread end bound: the thread stops (stays enabled, no IRQ)
 * when the PC equals END_ADDR, which is what the driver's clear-event poll
 * (CURR_ADDR == END_ADDR) and the trailing JUMP that cmdq_pkt_finalize()
 * appends rely on.  The done IRQ is latched only by an EOC instruction that
 * carries CMDQ_EOC_IRQ_EN; it is never synthesised when the PC reaches the end.
 *
 * A wait-for-event on an event the model actually generates parks the thread
 * (hardware behaviour, resumed by mtk_gce_event_signal()); a POLL mismatch, an
 * unknown opcode and a WFE on an event no modelled block raises are counted
 * and skipped, so a wrong decoding assumption can only lose a register update,
 * it can never hang the guest.
 */
static void mtk_gce_exec(MtkGceState *s, unsigned idx)
{
    MtkGceThread *t = &s->thr[idx];
    uint32_t pc = t->curr_addr;
    uint32_t end = t->end_addr;
    const uint32_t step = s->xlate_shift >= 3 ?
                          (uint32_t)(CMDQ_INST_SIZE >> 3) :
                          (uint32_t)(CMDQ_INST_SIZE >> s->xlate_shift);
    unsigned steps = 0;
    unsigned logged = 0;
    bool parked = false;
    const char *fault = NULL;
    uint64_t raw_ring[MTK_GCE_RAW_RING];
    uint32_t raw_pc_ring[MTK_GCE_RAW_RING];
    unsigned raw_n = 0;
    bool raw_active = false;
    unsigned raw_printed = 0;

    if (!pc) {
        return;
    }
    while (steps < MTK_GCE_MAX_STEPS) {
        uint64_t raw;
        uint32_t val;
        uint8_t op;
        unsigned sub, off;

        if (end && pc == end) {
            /* Hardware thread end: stay enabled and wait for the next task. */
            parked = true;
            break;
        }
        if (step > 1 && (pc & (step - 1))) {
            fault = "misaligned PC after jump";
            break;
        }
        if (!mtk_gce_fetch(s, pc, &raw)) {
            s->n_badfetch++;
            if (s->log && s->n_badfetch <= 16) {
                hwaddr pa = 0;

                if (!mtk_gce_xlate_to_pa(s, pc, &pa)) {
                    pa = 0;
                }
                fprintf(stderr,
                        "MTK-GCE thr%u: fetch failed pc=0x%08x pa=0x%" PRIx64
                        "\n", idx, pc, (uint64_t)pa);
            }
            break;
        }
        if (s->raw_log) {
            if (raw_active) {
                if (raw_printed++ < 4096) {
                    fprintf(stderr,
                            "MTK-GCE-RAW thr%u pc=%08x inst=%016" PRIx64 "\n",
                            idx, pc, raw);
                }
            } else {
                raw_ring[raw_n & (MTK_GCE_RAW_RING - 1)] = raw;
                raw_pc_ring[raw_n & (MTK_GCE_RAW_RING - 1)] = pc;
                raw_n++;
            }
        }
        op = raw >> 56;
        sub = (raw >> 48) & 0xff;
        off = (raw >> 32) & 0xffff;
        val = raw;
        steps++;
        s->n_inst++;
        s->opcnt[op]++;

        if (s->instlog && s->n_logged < 20000 && logged < 600 &&
            (t->irq_enable || s->n_started < 8)) {
            mtk_gce_log_inst(s, idx, pc, raw);
            s->n_logged++;
            logged++;
        }

        switch (op) {
        case CMDQ_CODE_MASK: {
            unsigned sop = (raw >> 48) & 0x1f;
            bool dst_t = (raw >> 55) & 1;

            if (dst_t) {
                /* cmdq_pkt_assign(): GPR[sop] = value */
                t->spr[sop & (MTK_GCE_SPR_NUM - 1)] = val;
            } else {
                /* cmdq_pkt_mask(): arg_a is ~mask for the next write/poll */
                t->mask = val;
                t->mask_valid = 1;
            }
            pc += step;
            break;
        }
        case CMDQ_CODE_LOGIC: {
            /*
             * LOGIC (vendor cmdq_pkt_assign / cmdq_pkt_logic), decoded from the
             * guest's own CMDQ dumps:
             *   bits 55..48 s_op: 0x80 = assign immediate, 0xeb = AND
             *   bits 47..32 destination GPR
             *   bits 31..16 GPR A (AND) / immediate high half (assign)
             *   bits 15..0  GPR B (AND) / immediate low half (assign)
             * The DRM ESD ready-poll packet uses assign GPR0 = 0x80000000
             * (mask) followed by "Reg2 = Reg2 & Reg0" (test bit 31); the old
             * assign-only model destroyed the value being tested.
             */
            unsigned dst = (raw >> 32) & 0xffff;
            unsigned s_op = (raw >> 48) & 0xff;

            if (s_op == 0x80) {
                t->spr[dst & (MTK_GCE_SPR_NUM - 1)] = val;
            } else if (s_op == 0xeb) {
                unsigned src_a = (raw >> 16) & 0xffff;
                unsigned src_b = raw & 0xffff;

                t->spr[dst & (MTK_GCE_SPR_NUM - 1)] =
                    t->spr[src_a & (MTK_GCE_SPR_NUM - 1)] &
                    t->spr[src_b & (MTK_GCE_SPR_NUM - 1)];
            } else if ((raw >> 55) & 1) {
                t->spr[dst & (MTK_GCE_SPR_NUM - 1)] = val;
            }
            pc += step;
            break;
        }
        case CMDQ_CODE_JUMPC: {
            /*
             * Conditional absolute jump (vendor cmdq_pkt_cond_jump_eq), decoded
             * from the guest dump "Jumpc absolute if (Reg Index 2 == Reg Index 1)
             * jump Reg Index 0":
             *   bits 47..32 target-address GPR
             *   bits 31..16 GPR A
             *   bits 15..0  GPR B
             * The DRM ESD packet uses it as the exit of its ready-poll loop.
             * Without it the model spun to the instruction step limit and raised
             * a false CMDQ error IRQ, which the kernel turned into the
             * "aee not ready" / cmdq_task_handle_error / emidbg flood.
             */
            unsigned reg_t = (raw >> 32) & 0xffff;
            unsigned reg_a = (raw >> 16) & 0xffff;
            unsigned reg_b = raw & 0xffff;
            uint32_t from = pc;
            uint32_t a = t->spr[reg_a & (MTK_GCE_SPR_NUM - 1)];
            uint32_t b = t->spr[reg_b & (MTK_GCE_SPR_NUM - 1)];

            if (a == b) {
                pc = t->spr[reg_t & (MTK_GCE_SPR_NUM - 1)];
            } else {
                pc += step;
            }
            s->n_jumps++;
            t->n_jumps++;
            if (s->log && s->n_logged < 20000 && logged < 600) {
                fprintf(stderr,
                        "MTK-GCE-I thr%u pc=%08x JUMPC %s a=%08x b=%08x -> %08x\n",
                        idx, from, a == b ? "taken" : "not", a, b, pc);
                s->n_logged++;
                logged++;
            }
            break;
        }
        case CMDQ_CODE_WRITE: {
            uint32_t pa = mtk_gce_subsys_addr(s, sub, off, NULL);
            uint32_t v = val;

            if ((off & 1) && t->mask_valid) {
                uint32_t cur = mtk_gce_read32(s, pa);

                v = (cur & t->mask) | (val & ~t->mask);
            }
            if (s->exec_write && pa) {
                mtk_gce_write32(s, pa, v);
                if (mtk_gce_pa_is_disp(pa)) {
                    mtk_gce_disp_epoch++;
                    if (s->raw_log) {
                        mtk_gce_raw_disp(idx, raw_ring, raw_pc_ring,
                                         raw_n, &raw_active);
                    }
                }
                if (mtk_gce_pa_is_disp(pa) && s->log) {
                    fprintf(stderr,
                            "MTK-GCE-DISP thr%u pc=%08x W 0x%08x = 0x%08x\n",
                            idx, pc, pa, v);
                }
            }
            pc += step;
            break;
        }
        case CMDQ_CODE_WRITE_S:
        case CMDQ_CODE_WRITE_S_MASK: {
            unsigned sop = (raw >> 48) & 0x1f;
            unsigned src = (raw >> 16) & 0xffff;
            bool src_t = (raw >> 54) & 1;
            uint32_t v = src_t ? t->spr[src & (MTK_GCE_SPR_NUM - 1)] : val;
            uint32_t pa = mtk_gce_gpr_addr(t, sop, off);
            /*
             * Masked-write semantics, exactly as the hardware decodes the two
             * opcodes: only CMDQ_CODE_WRITE_S_MASK (the instruction whose
             * encoding carries the mask flag) applies the value mask set by
             * the preceding MASK instruction; a plain WRITE_S always writes
             * its value unmasked.  arg_b bit1 is an address/mode flag, not a
             * mask enable (applying the mask to a plain write made the OVL
             * layer ADDR write 0xc0900870 collapse to 0 under mask
             * 0xfffffff8).  The MASK instruction carries ~mask in arg_a, so:
             *   reg = (reg & mask) | (value & ~mask)
             */
            bool use_mask = op == CMDQ_CODE_WRITE_S_MASK;

            if (use_mask && t->mask_valid) {
                uint32_t cur = mtk_gce_read32(s, pa);

                v = (cur & t->mask) | (v & ~t->mask);
            }
            if (s->exec_write && pa) {
                mtk_gce_write32(s, pa, v);
                if (mtk_gce_pa_is_disp(pa)) {
                    mtk_gce_disp_epoch++;
                    if (s->raw_log) {
                        mtk_gce_raw_disp(idx, raw_ring, raw_pc_ring,
                                         raw_n, &raw_active);
                    }
                }
                if (mtk_gce_pa_is_disp(pa) && s->log) {
                    fprintf(stderr,
                            "MTK-GCE-DISP thr%u pc=%08x WS 0x%08x = 0x%08x "
                            "(spr[%u]=%08x arg_b=%04x masked=%d mask=%08x)\n",
                            idx, pc, pa, v, sop,
                            t->spr[sop & (MTK_GCE_SPR_NUM - 1)],
                            off, use_mask && t->mask_valid,
                            t->mask_valid ? t->mask : 0);
                }
            }
            pc += step;
            break;
        }
        case CMDQ_CODE_READ_S: {
            unsigned sop = (raw >> 48) & 0x1f;
            unsigned dst = (raw >> 32) & 0xffff;
            uint32_t pa = mtk_gce_gpr_addr(t, sop, (raw & 0xffff) & 0xfffcu);

            t->spr[dst & (MTK_GCE_SPR_NUM - 1)] = mtk_gce_read32(s, pa);
            pc += step;
            break;
        }
        case CMDQ_CODE_POLL: {
            bool known;
            uint32_t pa = mtk_gce_subsys_addr(s, (raw >> 48) & 0xff, off, &known);
            uint32_t cur = mtk_gce_read32(s, pa);
            uint32_t mask = (off & 1) && t->mask_valid ? ~t->mask : ~0u;

            if ((cur & mask) != (val & mask)) {
                s->n_pollfail++;
                if (s->log && s->n_pollfail <= 32) {
                    fprintf(stderr,
                            "MTK-GCE thr%u pc=%08x POLL 0x%08x cur=%08x "
                            "want=%08x mask=%08x%s (skip)\n",
                            idx, pc, pa, cur, val, mask,
                            known ? "" : " no-subsys");
                }
            }
            pc += step;
            break;
        }
        case CMDQ_CODE_WFE: {
            unsigned event = off;
            bool update = (val & CMDQ_WFE_UPDATE) != 0;
            bool update_value = (val & CMDQ_WFE_UPDATE_VALUE) != 0;
            bool wait = (val & CMDQ_WFE_WAIT) != 0;
            bool wait_value = (val & CMDQ_WFE_WAIT_VALUE) != 0;

            s->n_events++;

            if (!wait) {
                /* set_event / clear_event: update the bit immediately. */
                if (update) {
                    mtk_gce_event_put(s, event, update_value);
                }
                pc += step;
                break;
            }

            if (!mtk_gce_event_is_generated(s, event)) {
                /*
                 * No modelled block raises this event: skip the wait so a
                 * missing event model cannot hang the commit path.  Each such
                 * event id is logged once.
                 */
                s->n_wfe_skipped++;
                if (s->wfe_log && event < MTK_GCE_NUM_EVENTS &&
                    !(s->event_skip_logged[event >> 3] &
                      (1u << (event & 7)))) {
                    s->event_skip_logged[event >> 3] |= 1u << (event & 7);
                    fprintf(stderr,
                            "MTK-GCE thr%u: WFE event %u cannot be generated"
                            " by the model (skip, guest=%0.3fs)\n",
                            idx, event, mtk_gce_guest_seconds());
                }
                pc += step;
                break;
            }

            if (mtk_gce_event_isset(s, event) != wait_value) {
                /* Park the thread on the event, exactly like the GCE. */
                t->curr_addr = pc + step;
                s->wfe_waiting[idx] = true;
                s->wfe_event[idx] = event;
                s->wfe_clear[idx] = update && !update_value;
                s->n_wfe_blocked++;
                if (s->wfe_log &&
                    (s->n_wfe_blocked <= 32 ||
                     (s->n_wfe_blocked % 256) == 0)) {
                    fprintf(stderr,
                            "MTK-GCE thr%u: WFE block event %u clear=%d"
                            " (n=%u guest=%0.3fs)\n",
                            idx, event, s->wfe_clear[idx],
                            s->n_wfe_blocked, mtk_gce_guest_seconds());
                }
                return;
            }

            /* The event was already set: consume it like hardware does. */
            if (update) {
                mtk_gce_event_put(s, event, update_value);
            }
            pc += step;
            break;
        }
        case CMDQ_CODE_JUMP: {
            uint32_t from = pc;

            /*
             * The target type is arg_a (instruction bits 32..47):
             *   1 -> absolute: arg_b:arg_c is the target GCE address, i.e.
             *        CMDQ_REG_SHIFT_ADDR(pa) as written by cmdq_pkt_jump_addr()
             *        and by cmdq_task_connect_buffer()'s
             *        (CMDQ_JUMP_BY_PA << 32) | SHIFT_ADDR(next_pa) chunk link;
             *   0 -> relative: arg_b:arg_c is a signed offset already shifted
             *        to GCE address units by cmdq_pkt_jump(offset >> shift).
             * The PC stays in GCE address space; mtk_gce_fetch() reverts it to
             * (pc << shift) - gce_mminfra before reading guest memory, so an
             * absolute jump lands exactly on CMDQ_REG_REVERT_ADDR(arg).
             */
            if (off == 1) {
                pc = val;
            } else {
                pc += (int32_t)val;
            }
            s->n_jumps++;
            t->n_jumps++;
            if (s->log && s->n_logged < 20000 && logged < 600) {
                fprintf(stderr,
                        "MTK-GCE-I thr%u pc=%08x JUMP arg_a=%u %s -> %08x\n",
                        idx, from, off, off == 1 ? "abs" : "rel", pc);
                s->n_logged++;
                logged++;
            }
            if (end && pc == end) {
                /* The finalize() "JUMP pass" over END_ADDR parks here. */
                parked = true;
                goto out;
            }
            break;
        }
        case CMDQ_CODE_EOC: {
            /* The IRQ flag is arg_c bit0 (CMDQ_EOC_IRQ_EN), not arg_a. */
            if (val & CMDQ_EOC_IRQ_EN) {
                mtk_gce_eoc_irq(s, idx);
                t->n_eoc++;
                s->n_eoc++;
                if (s->log &&
                    (s->n_eoc <= 4096 || (s->n_eoc % 4096) == 0)) {
                    fprintf(stderr,
                            "MTK-GCE EOC thr%u pc=%08x n_thr=%u n=%" PRIu64
                            " guest=%.3fs\n",
                            idx, pc, t->n_eoc, s->n_eoc,
                            mtk_gce_guest_seconds());
                }
            }
            /*
             * EOC only marks the end of a command group: the thread keeps
             * fetching the trailing JUMP that cmdq_pkt_finalize() appends and
             * then parks on END_ADDR, exactly like the hardware.
             */
            pc += step;
            break;
        }
        default:
            s->n_unknown++;
            if (s->log && s->n_unknown <= 32) {
                fprintf(stderr,
                        "MTK-GCE thr%u pc=%08x unknown opcode 0x%02x "
                        "inst=%016" PRIx64 " (skip)\n", idx, pc, op, raw);
            }
            pc += step;
            break;
        }
    }
    if (steps >= MTK_GCE_MAX_STEPS) {
        s->n_steplimit++;
        fault = "instruction step limit";
    }

out:
    /*
     * The architectural PC is what the driver reads back through
     * CMDQ_THR_CURR_ADDR and reverts with CMDQ_REG_REVERT_ADDR().  A thread
     * that reached END_ADDR stays enabled and idle; it reports completion only
     * through the EOC IRQ latched above.
     */
    t->curr_addr = pc;
    if (parked) {
        s->n_started++;
        if (s->log && (t->irq_status & THR_IRQ_DONE) &&
            s->n_parked_log < 64) {
            s->n_parked_log++;
            fprintf(stderr,
                    "MTK-GCE thr%u packet parked at end pc=0x%08x"
                    " end=0x%08x after %u steps eoc=%u guest=%.3fs\n",
                    idx, pc, end, steps, t->n_eoc, mtk_gce_guest_seconds());
        }
    }
    if (fault) {
        if (s->log) {
            fprintf(stderr,
                    "MTK-GCE thr%u stopped: %s pc=0x%08x end=0x%08x\n",
                    idx, fault, pc, end);
        }
        mtk_gce_error_irq(s, idx);
    }
}

/*
 * Wake every thread parked in a WFE on @event.  Hardware releases all waiters
 * when the event bit becomes set; a wait-and-clear WFE consumes the bit once
 * and every released thread continues at the instruction after its WFE.
 */
static void mtk_gce_wake_event_waiters(MtkGceState *s, unsigned event)
{
    unsigned idx;
    uint32_t wake = 0;
    bool clear = false;

    if (event >= MTK_GCE_NUM_EVENTS) {
        return;
    }
    for (idx = 0; idx < MTK_GCE_NUM_THREADS; idx++) {
        if (!s->wfe_waiting[idx] || s->wfe_event[idx] != event) {
            continue;
        }
        /*
         * A suspended thread must not run: it keeps its wait until the driver
         * resumes it (the event bit stays set for a wait-and-clear WFE).
         */
        if (s->thr[idx].suspended) {
            continue;
        }
        wake |= 1u << idx;
        clear |= s->wfe_clear[idx];
    }
    if (!wake) {
        return;
    }
    if (clear) {
        mtk_gce_event_put(s, event, false);
    }
    for (idx = 0; idx < MTK_GCE_NUM_THREADS; idx++) {
        if (!(wake & (1u << idx))) {
            continue;
        }
        s->wfe_waiting[idx] = false;
        s->n_wfe_resumed++;
        if (s->wfe_log &&
            (s->n_wfe_resumed <= 32 || (s->n_wfe_resumed % 256) == 0)) {
            fprintf(stderr,
                    "MTK-GCE thr%u: WFE resume event %u (n=%u guest=%0.3fs)\n",
                    idx, event, s->n_wfe_resumed, mtk_gce_guest_seconds());
        }
        mtk_gce_exec(s, idx);
    }
}

void mtk_gce_event_signal(struct DeviceState *dev, unsigned event)
{
    MtkGceState *s = MTK_GCE(OBJECT(dev));

    mtk_gce_event_put(s, event, true);
    mtk_gce_wake_event_waiters(s, event);
}

void mtk_gce_event_mark_generated(struct DeviceState *dev, unsigned event)
{
    mtk_gce_event_mark(MTK_GCE(OBJECT(dev)), event);
}

static void mtk_gce_start(MtkGceState *s, unsigned idx)
{
    MtkGceThread *t = &s->thr[idx];

    if (!s->exec) {
        mtk_gce_thread_complete(s, idx);
        return;
    }
    mtk_gce_calibrate(s, t->curr_addr);
    if (s->log && s->n_started <= 64) {
        fprintf(stderr,
                "MTK-GCE run thr%u gce=0x%08x end=0x%08x irq_en=0x%x %s "
                "(mminfra=0x%x shift=%u)\n",
                idx, t->curr_addr, t->end_addr, t->irq_enable,
                s->exec_write ? "exec" : "dump", s->xlate_off, s->xlate_shift);
    }
    mtk_gce_exec(s, idx);
    if (s->log && (s->n_started % 32) == 0) {
        mtk_gce_dump_ops(s);
    }
}

/*
 * MTK_GCE_REGLOG=1: trace the thread register traffic the driver performs.
 * Used to see whether a CMDQ task submission reaches the GCE (CURR_ADDR /
 * END_ADDR / IRQ_ENABLE / ENABLE_TASK writes) when the guest stalls.
 */
static void mtk_gce_reglog(MtkGceState *s, const char *rw, unsigned idx,
                           unsigned reg, uint32_t val)
{
    if (!s->reg_log || s->n_reg_logged >= 4000) {
        return;
    }
    s->n_reg_logged++;
    fprintf(stderr, "MTK-GCE-REG %s thr%u off=%02x val=%08x guest=%.3fs\n",
            rw, idx, reg, val, mtk_gce_guest_seconds());
}

static uint64_t mtk_gce_read(void *opaque, hwaddr addr, unsigned size)
{
    MtkGceState *s = opaque;
    unsigned idx, reg;
    MtkGceThread *t;

    addr &= MTK_GCE_SIZE - 1;
    if (mtk_gce_thread_lookup(addr, &idx, &reg)) {
        t = &s->thr[idx];
        switch (reg) {
        case THR_WARM_RESET:
            mtk_gce_reglog(s, "R", idx, reg, 0);
            return 0;   /* self-clearing */
        case THR_ENABLE_TASK:
            mtk_gce_reglog(s, "R", idx, reg, t->enable);
            return t->enable;
        case THR_SUSPEND_TASK:
            mtk_gce_reglog(s, "R", idx, reg, t->suspended);
            return t->suspended;
        case THR_CURR_STATUS: {
            uint32_t st = t->suspended ? THR_STATUS_SUSPENDED : 0;

            mtk_gce_reglog(s, "R", idx, reg, st);
            return st;
        }
        case THR_IRQ_STATUS:
            mtk_gce_reglog(s, "R", idx, reg, t->irq_status);
            return t->irq_status;
        case THR_IRQ_ENABLE:
            mtk_gce_reglog(s, "R", idx, reg, t->irq_enable);
            return t->irq_enable;
        case THR_CURR_ADDR:
            mtk_gce_reglog(s, "R", idx, reg, t->curr_addr);
            return t->curr_addr;
        case THR_END_ADDR:
            mtk_gce_reglog(s, "R", idx, reg, t->end_addr);
            return t->end_addr;
        case THR_WAIT_TOKEN:
            mtk_gce_reglog(s, "R", idx, reg, 0);
            return 0;   /* never blocked on a token */
        case THR_PRIORITY:
            return t->priority;
        default:
            break;
        }
    }
    if (addr == GCE_CURR_IRQ_STATUS) {
        return ~s->pending & MTK_GCE_IRQ_MASK;
    }
    return s->regs[addr >> 2];
}

static void mtk_gce_write(void *opaque, hwaddr addr, uint64_t val,
                          unsigned size)
{
    MtkGceState *s = opaque;
    unsigned idx, reg;
    MtkGceThread *t;

    addr &= MTK_GCE_SIZE - 1;
    if (mtk_gce_thread_lookup(addr, &idx, &reg)) {
        t = &s->thr[idx];
        mtk_gce_reglog(s, "W", idx, reg, (uint32_t)val);
        switch (reg) {
        case THR_WARM_RESET:
            if (val & THR_DO_WARM_RESET) {
                mtk_gce_thread_reset(s, idx);
            }
            return;     /* self-clearing command register */
        case THR_ENABLE_TASK:
            if ((val & THR_ENABLED) && !t->enable) {
                t->enable = THR_ENABLED;
                if (t->end_addr) {
                    mtk_gce_start(s, idx);
                }
            } else if (!(val & THR_ENABLED)) {
                t->enable = 0;
            }
            return;
        case THR_SUSPEND_TASK: {
            bool was = t->suspended;

            t->suspended = (val & 1) ? 1 : 0;
            /*
             * cmdq_mbox_send_data() append path: the driver suspends the
             * thread, programs CURR_ADDR/END_ADDR for the *next* task and then
             * resumes it - it does not touch ENABLE_TASK, so an executor that
             * only runs on the 0->1 enable transition never sees any packet
             * after the first one (measured: exactly 5 thread runs per boot,
             * then nothing even though recovery issued atomic commits).
             * Treat a 1->0 resume on an enabled thread as "run the newly
             * queued task".
             */
            if (was && !t->suspended && !s->wfe_waiting[idx] &&
                t->enable && t->end_addr &&
                t->curr_addr && t->curr_addr != t->end_addr) {
                mtk_gce_start(s, idx);
            }
            return;
        }
        case THR_CURR_STATUS:
            return;     /* read-only */
        case THR_IRQ_STATUS:
            /*
             * Write-zero-to-clear status, exactly like the vendor GCE: the
             * driver acknowledges with writel(~irq_flag), so every bit it saw
             * is written 0 (cleared) and every other bit is written 1 (kept).
             * Implementing this as write-1-to-clear leaves THR_IRQ_STATUS.DONE
             * asserted forever: the level IRQ then storms and the guest
             * livelocks in cmdq_thread_irq_handler().
             */
            t->irq_status &= (uint32_t)val;
            mtk_gce_refresh_pending(s);
            return;
        case THR_IRQ_ENABLE:
            t->irq_enable = val;
            mtk_gce_refresh_pending(s);
            return;
        case THR_CURR_ADDR:
            t->curr_addr = val;
            return;
        case THR_END_ADDR:
            t->end_addr = val;
            return;
        case THR_WAIT_TOKEN:
            return;     /* read-only status */
        case THR_PRIORITY:
            t->priority = val;
            return;
        default:
            break;
        }
    }
    s->regs[addr >> 2] = val;
}

static const MemoryRegionOps mtk_gce_ops = {
    .read = mtk_gce_read,
    .write = mtk_gce_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void mtk_gce_reset(DeviceState *dev)
{
    MtkGceState *s = MTK_GCE(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->thr, 0, sizeof(s->thr));
    memset(s->event_bits, 0, sizeof(s->event_bits));
    memset(s->wfe_waiting, 0, sizeof(s->wfe_waiting));
    s->pending = 0;
    s->n_started = 0;
    mtk_gce_update_irq(s);
}

static void mtk_gce_parse_subsys(MtkGceState *s, const char *spec)
{
    const char *p = spec;

    while (p && *p) {
        char *end;
        unsigned long id = strtoul(p, &end, 0);
        unsigned long base;

        if (end == p || *end != ':') {
            break;
        }
        base = strtoul(end + 1, &end, 0);
        if (id < 256) {
            s->subsys[id] = (uint32_t)base;
        }
        p = *end == ',' ? end + 1 : NULL;
    }
}

static void mtk_gce_init(Object *obj)
{
    MtkGceState *s = MTK_GCE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    const char *e;
    const char *ss;

    memory_region_init_io(&s->iomem, obj, &mtk_gce_ops, s, "mt6895.gce",
                          MTK_GCE_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->log = getenv("MTK_GCELOG") != NULL;
    s->instlog = s->log;
    /*
     * MTK_GCE_RAW=1: dump the raw 64-bit instruction words of every packet
     * that writes a DISP register window, so the model's decoding can be
     * compared 1:1 with the driver's cmdq_log output.
     */
    e = getenv("MTK_GCE_RAW");
    s->raw_log = e != NULL && strcmp(e, "0") != 0;
    s->reg_log = getenv("MTK_GCE_REGLOG") != NULL;
    s->exec = true;
    s->exec_write = true;
    /*
     * Blocking WFE: a wait-for-event parks the thread until a peripheral
     * model raises the event (DSI0 EOF/SOF, DISP mutex0 EOF), exactly like
     * the hardware.  A WFE on an event no modelled block can raise keeps the
     * "count and skip" fallback in mtk_gce_exec(), so a missing event model
     * can never hang a guest commit packet.  MTK_DSITRACE=1 additionally logs
     * WFE block/resume, because those events gate the DRM page-flip
     * completion.
     */
    s->wfe_log = getenv("MTK_DSITRACE") != NULL ||
                 getenv("MTK_GCE_WFE_LOG") != NULL;
    e = getenv("MTK_GCE_EXEC");
    if (e && !strcmp(e, "off")) {
        s->exec = false;
    } else if (e && !strcmp(e, "dump")) {
        s->exec_write = false;
    }
    /*
     * Vendor device tree (gce-subsys in /soc/dispsys_config@14000000):
     *   id 1 -> 0x14000000, id 2 -> 0x14010000, id 3 -> 0x14020000.
     */
    s->subsys[1] = 0x14000000u;
    s->subsys[2] = 0x14010000u;
    s->subsys[3] = 0x14020000u;
    ss = getenv("MTK_GCE_SUBSYS");
    if (ss) {
        mtk_gce_parse_subsys(s, ss);
    }
    /*
     * MTK_GCE_MMINFRA=<hex>: force the GCE-address -> guest-PA offset
     * (pa = gce_addr - offset).  Default: calibrated on the first packet.
     */
    /*
     * Vendor mt6895 GCE address conversion.  The driver prints it at probe
     * ("cmdq thread:32 shift:3 mminfra:0x40000000") and mtk-cmdq-drv-ext.ko
     * shows the thread-dump path reverting an address with
     *   dma = (gce << shift) - mminfra
     * so the forward conversion is
     *   gce = (dma + mminfra) >> shift
     * (pa = (gce << shift) - mminfra).  MTK_GCE_XLATE=pre selects the older
     * mt8183/mt8192 style conversion (pa = (gce - mminfra) << shift) and
     * MTK_GCE_CALIBRATE=1 re-enables the first-packet auto-calibration.
     */
    s->xlate_off = 0x40000000u;
    s->xlate_shift = 3;
    s->xlate_post = true;
    e = getenv("MTK_GCE_XLATE");
    if (e && !strcmp(e, "pre")) {
        s->xlate_post = false;
    }
    s->xlate_done = getenv("MTK_GCE_CALIBRATE") == NULL;
    e = getenv("MTK_GCE_MMINFRA");
    if (e) {
        s->xlate_off = strtoul(e, NULL, 0);
        s->xlate_done = true;
        s->xlate_fixed = true;
    }
    e = getenv("MTK_GCE_SHIFT");
    if (e) {
        s->xlate_shift = strtoul(e, NULL, 0);
        s->xlate_done = true;
        s->xlate_fixed = true;
    }
}

static const VMStateDescription vmstate_mtk_gce_thread = {
    .name = "mtk-gce-thread",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(curr_addr, MtkGceThread),
        VMSTATE_UINT32(end_addr, MtkGceThread),
        VMSTATE_UINT32(enable, MtkGceThread),
        VMSTATE_UINT32(suspended, MtkGceThread),
        VMSTATE_UINT32(irq_status, MtkGceThread),
        VMSTATE_UINT32(irq_enable, MtkGceThread),
        VMSTATE_UINT32(priority, MtkGceThread),
        VMSTATE_UINT32_ARRAY(spr, MtkGceThread, MTK_GCE_SPR_NUM),
        VMSTATE_UINT32(mask, MtkGceThread),
        VMSTATE_UINT32(mask_valid, MtkGceThread),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_mtk_gce = {
    .name = TYPE_MTK_GCE,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, MtkGceState, MTK_GCE_SIZE / 4),
        VMSTATE_STRUCT_ARRAY(thr, MtkGceState, MTK_GCE_NUM_THREADS, 2,
                             vmstate_mtk_gce_thread, MtkGceThread),
        VMSTATE_UINT32(pending, MtkGceState),
        VMSTATE_UINT32(n_started, MtkGceState),
        VMSTATE_END_OF_LIST()
    },
};

static void mtk_gce_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, mtk_gce_reset);
    dc->vmsd = &vmstate_mtk_gce;
}

static const TypeInfo mtk_gce_info = {
    .name = TYPE_MTK_GCE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MtkGceState),
    .instance_init = mtk_gce_init,
    .class_init = mtk_gce_class_init,
};

static void mtk_gce_register_types(void)
{
    type_register_static(&mtk_gce_info);
}

type_init(mtk_gce_register_types)
