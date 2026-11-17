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
 *   WFE:    event wait/set - recorded but never blocking
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
#include "system/memory.h"
#include "hw/misc/mtk_iommu.h"

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
    /*
     * Value-mask semantics for opcode 0x90 with arg_b bit1 set:
     *   0 = MTK_GCE_MASK_NONE  do not apply the MASK register
     *   1 = forward (default, hardware-correct): new = (old & arg_a) | (val & ~arg_a)
     *   2 = MTK_GCE_MASK_INV   inverted:          new = (old & ~arg_a) | (val & arg_a)
     * The forward form is what reproduces the vendor display driver's masked
     * ROI write (0x14002020 -> 0x099c0209, 540x2460) on LineageOS; the
     * inverted form is kept for the stock-kernel ADDR writes (env-protected).
     */
    int mask_mode;
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
            " unknown=%u badfetch=%u pollfail=%u events=%u\n",
            s->opcnt[CMDQ_CODE_MASK], s->opcnt[CMDQ_CODE_WRITE],
            s->opcnt[CMDQ_CODE_POLL], s->opcnt[CMDQ_CODE_JUMP],
            s->opcnt[CMDQ_CODE_WFE], s->opcnt[CMDQ_CODE_EOC],
            s->opcnt[CMDQ_CODE_WRITE_S] + s->opcnt[CMDQ_CODE_WRITE_S_MASK],
            s->opcnt[CMDQ_CODE_READ_S], s->opcnt[CMDQ_CODE_LOGIC],
            s->opcnt[0], s->n_inst, s->n_unknown, s->n_badfetch,
            s->n_pollfail, s->n_events);
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
    if (t->irq_enable & THR_IRQ_DONE) {
        s->pending |= 1u << idx;
        mtk_gce_update_irq(s);
    }
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
 * Never blocks: polls that do not match, unknown opcodes and wait-for-event
 * instructions are counted and skipped, so a wrong decoding assumption can
 * only lose a register update - it can never hang the guest.
 */
static void mtk_gce_exec(MtkGceState *s, unsigned idx)
{
    MtkGceThread *t = &s->thr[idx];
    uint32_t pc = t->curr_addr;
    uint32_t end = t->end_addr;
    unsigned steps = 0;
    unsigned logged = 0;
    bool completed = false;

    if (!pc) {
        return;
    }
    while (steps < MTK_GCE_MAX_STEPS) {
        uint64_t raw;
        uint32_t val;
        uint8_t op;
        unsigned sub, off;

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
            pc += 8;
            break;
        }
        case CMDQ_CODE_LOGIC: {
            /* GPR assignment / logic op; only the plain assign is needed. */
            unsigned dst = (raw >> 32) & 0xffff;
            bool dst_t = (raw >> 55) & 1;

            if (dst_t) {
                t->spr[dst & (MTK_GCE_SPR_NUM - 1)] = val;
            }
            pc += 8;
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
                if (pa >= 0x14000000u && pa < 0x14500000u) {
                    mtk_gce_disp_epoch++;
                }
                if (pa >= 0x14000000u && pa < 0x14500000u && s->log) {
                    fprintf(stderr,
                            "MTK-GCE-DISP thr%u pc=%08x W 0x%08x = 0x%08x\n",
                            idx, pc, pa, v);
                }
            }
            pc += 8;
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
             * Masked write semantics: CMDQ_CODE_WRITE_S_MASK always uses the
             * mask set by the preceding MASK instruction; a plain WRITE_S
             * uses it when arg_b bit1 (the "write enable" flag) is set.
             * reg = (reg & ~mask) | (value & mask), with the MASK
             * instruction carrying ~mask in arg_a.
             */
            /*
             * Masked-write selection.  Vendor display packets use opcode
             * 0x91 (WRITE_S_MASK) for bitfield updates and 0x90 for plain
             * writes; arg_b bit1 is an address/mode flag, NOT a mask enable
             * (masking 0x90 made the OVL ADDR write 0xc0900870 collapse to
             * 0 under mask=0xfffffff8).  MTK_GCE_MASK_ALL=1 restores the
             * over-eager semantics for A/B comparison.
             */
            bool use_mask = (op == CMDQ_CODE_WRITE_S_MASK) ||
                            (s->mask_mode != 0 && ((off & 2) != 0));

            if (use_mask && t->mask_valid) {
                uint32_t cur = mtk_gce_read32(s, pa);

                if (s->mask_mode == 2) {
                    /* inverted: arg_a is the "update" mask */
                    v = (cur & ~t->mask) | (v & t->mask);
                } else {
                    v = (cur & t->mask) | (v & ~t->mask);
                }
            }
            if (s->exec_write && pa) {
                mtk_gce_write32(s, pa, v);
                if (pa >= 0x14000000u && pa < 0x14500000u) {
                    mtk_gce_disp_epoch++;
                }
                if (pa >= 0x14000000u && pa < 0x14500000u && s->log) {
                    fprintf(stderr,
                            "MTK-GCE-DISP thr%u pc=%08x WS 0x%08x = 0x%08x "
                            "(spr[%u]=%08x arg_b=%04x masked=%d mask=%08x)\n",
                            idx, pc, pa, v, sop,
                            t->spr[sop & (MTK_GCE_SPR_NUM - 1)],
                            off, use_mask && t->mask_valid,
                            t->mask_valid ? t->mask : 0);
                }
            }
            pc += 8;
            break;
        }
        case CMDQ_CODE_READ_S: {
            unsigned sop = (raw >> 48) & 0x1f;
            unsigned dst = (raw >> 32) & 0xffff;
            uint32_t pa = mtk_gce_gpr_addr(t, sop, (raw & 0xffff) & 0xfffcu);

            t->spr[dst & (MTK_GCE_SPR_NUM - 1)] = mtk_gce_read32(s, pa);
            pc += 8;
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
            pc += 8;
            break;
        }
        case CMDQ_CODE_WFE: {
            s->n_events++;
            pc += 8;
            break;
        }
        case CMDQ_CODE_JUMP: {
            if (off == 1) {
                /* cmdq_pkt_jump_abs(): arg_a is the target PA >> shift_pa */
                pc = val;
            } else if (val & 0x10000000u) {
                /* legacy CMDQ_JUMP_BY_OFFSET / CMDQ_JUMP_BY_PA encoding */
                if (val & 1u) {
                    pc = val & 0x0ffffffcu;
                } else {
                    pc += (int32_t)(val & 0x0ffffffcu);
                }
            } else {
                /* cmdq_pkt_jump_rel(): arg_a is a relative offset */
                pc += (int32_t)val;
            }
            if (s->log && s->n_logged < 20000 && logged < 600) {
                fprintf(stderr, "MTK-GCE-I thr%u pc=%08x JUMP -> %08x\n",
                        idx, pc, pc);
                logged++;
            }
            break;
        }
        case CMDQ_CODE_EOC: {
            if (val & 1u) {
                mtk_gce_thread_complete(s, idx);
                completed = true;
                goto out;
            }
            /* End of a command group without an interrupt: keep going. */
            pc += 8;
            break;
        }
        default:
            s->n_unknown++;
            if (s->log && s->n_unknown <= 32) {
                fprintf(stderr,
                        "MTK-GCE thr%u pc=%08x unknown opcode 0x%02x "
                        "inst=%016" PRIx64 " (skip)\n", idx, pc, op, raw);
            }
            pc += 8;
            break;
        }

        if (op != CMDQ_CODE_JUMP && end && pc > end) {
            break;
        }
    }

    if (!completed) {
        if (s->log && s->n_started < 64) {
            fprintf(stderr,
                    "MTK-GCE thr%u pc=0x%08x end=0x%08x reached end after %u "
                    "steps (no EOC-irq)\n", idx, pc, end, steps);
        }
        mtk_gce_thread_complete(s, idx);
    }
out:
    /* The driver polls CURR_ADDR == END_ADDR after enabling a task. */
    t->curr_addr = end;
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
            return 0;   /* self-clearing */
        case THR_ENABLE_TASK:
            return t->enable;
        case THR_SUSPEND_TASK:
            return t->suspended;
        case THR_CURR_STATUS:
            return t->suspended ? THR_STATUS_SUSPENDED : 0;
        case THR_IRQ_STATUS:
            return t->irq_status;
        case THR_IRQ_ENABLE:
            return t->irq_enable;
        case THR_CURR_ADDR:
            return t->curr_addr;
        case THR_END_ADDR:
            return t->end_addr;
        case THR_WAIT_TOKEN:
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
            if (was && !t->suspended && t->enable && t->end_addr &&
                t->curr_addr && t->curr_addr != t->end_addr) {
                mtk_gce_start(s, idx);
            }
            return;
        }
        case THR_CURR_STATUS:
            return;     /* read-only */
        case THR_IRQ_STATUS:
            t->irq_status &= ~(uint32_t)val;    /* write-1-to-clear */
            if (s->pending & (1u << idx)) {
                s->pending &= ~(1u << idx);
                mtk_gce_update_irq(s);
            }
            return;
        case THR_IRQ_ENABLE:
            t->irq_enable = val;
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
    s->exec = true;
    s->exec_write = true;
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
    /*
     * Default = forward (vendor display masked-write semantics, verified with
     * the LineageOS kernel's 0x14002020 ROI write in round M1.5).  The stock
     * kernel's OVL ADDR writes need the inverted form, so those runs set
     * MTK_GCE_MASK_INV=1; MTK_GCE_MASK_NONE=1 restores the old "do not mask
     * opcode 0x90" behaviour for A/B testing.
     */
    s->mask_mode = 0;                      /* default: stock path unchanged */
    if (getenv("MTK_GCE_MASK_ALL")) {
        s->mask_mode = 1;                  /* forward: LOS/vendor masked writes */
    }
    if (getenv("MTK_GCE_MASK_INV")) {
        s->mask_mode = 2;                  /* inverted: stock OVL ADDR writes */
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
