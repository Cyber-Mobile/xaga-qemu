/*
 * MediaTek SCPSYS power-domain controller
 *
 * scpsys@1c001000 gates the SoC's power domains (mtcmos). The ROM kernel's
 * mtk-scpsys driver drives it, and UFS needs its ufs0_shutdown domain from
 * here, so a block that never reports a domain as up keeps UFS down.
 *
 * The register block used to be a plain read/write file, and a trace of the
 * window showed why that is not enough: the driver wrote the domain's
 * power-on bits and then read the same register 6.9 million times waiting for
 * something the echo never produced, so every domain timed out.
 *
 * What the poll waits for was found by sweeping the value a read returns,
 * using "does mm_infra retry" as the signal (67 retries at baseline, 0 when
 * satisfied):
 *
 *   any single bit in 0..29      -> still retries
 *   BIT(30) alone                -> still retries
 *   BIT(31) alone                -> still retries
 *   BIT(30) | BIT(31)            -> poll completes
 *
 * Both bits together satisfy it and nothing else does, so they are reported
 * on every register of the block, including before the guest writes anything.
 * Driving them only after a power-on request -- the obvious "acknowledge"
 * reading -- leaves the boot worse than the plain echo did, so they are not
 * an acknowledgement of the write; they belong to the block itself.
 *
 * Access widths follow mtk-syscon: the block answers byte, halfword and word
 * accesses, because the guest is not restricted to word-wide ones.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "hw/core/cpu.h"
#include "trace.h"

#define TYPE_MTK_SCPSYS "mtk-scpsys"
OBJECT_DECLARE_SIMPLE_TYPE(MtkScpsysState, MTK_SCPSYS)

/*
 * Power-domain control (PWR_CON) registers, from the vendor
 * drivers/soc/mediatek/mtk-scpsys-mt6895.c scp_domain_data[] table: those
 * are the ctl_offs values of every domain driven by the generic
 * scpsys_power_on()/scpsys_power_off() flow.
 *
 * A PWR_CON word is a state machine, not a status bit that can be invented:
 *
 *   bit0  PWR_RST_B          software control (kept as written)
 *   bit1  PWR_ISO            software control
 *   bit2  PWR_ON             request: subsys power on
 *   bit3  PWR_ON_2ND         request: second power switch
 *   bit4  PWR_CLK_DIS        software control
 *   bit5  PWR_SRAM_CLKISO    software control
 *   bit6  PWR_SRAM_ISOINT_B  software control
 *   bit8  SRAM_PDN           request: SRAM power-down
 *   bit9  SRAM_SLP           request: SRAM sleep
 *   bit12 SRAM_PDN_ACK       output: SRAM macro acked the power-down
 *   bit13 SRAM_SLP_ACK       output: SRAM macro acked the sleep request
 *   bit30 PWR_ACK            output: subsys reports powered
 *   bit31 PWR_ACK_2ND        output: second power switch acknowledged
 *
 * The acknowledge bits are therefore derived from the request bits the guest
 * last drove, and the status bits only after both PWR_ON bits were set.  The
 * vendor driver polls exactly these (scpsys_sram_on/off() for bit12/13,
 * scpsys_pwr_con_is_on() for bits 30/31), so a constant or a plain echo
 * breaks its power sequencing (-ETIMEDOUT, then a genpd rollback notifier
 * that pd_chk_mt6895 turns into a BUG).
 *
 * The window is shared: the SPM (vmmspm, "mediatek,vmm_spm") maps the same
 * 0x1c001000 block and uses registers 0x69c..0x6b8 for hardware semaphores
 * plus 0xf30 for its own status; those are kept out of the PWR_CON set.
 */
#define SCPSYS_STATUS_MASK      0xc0000000      /* BIT(30) | BIT(31) */
#define SCPSYS_WRITABLE_MASK    0x00000fff      /* RST_B .. SRAM_SLP      */
#define PWR_ON_BIT              BIT(2)
#define PWR_ON_2ND_BIT          BIT(3)
#define SCPSYS_SRAM_PDN_BIT     BIT(8)
#define SCPSYS_SRAM_PDN_ACK     BIT(12)
#define SCPSYS_SRAM_SLP_BIT     BIT(9)
#define SCPSYS_SRAM_SLP_ACK     BIT(13)
/* SPM power-status words (mtk-scpsys-mt6895.c scp_soc_data.regs). */
#define SCPSYS_PWR_STATUS_OFF   0x0f34
#define SCPSYS_PWR_STATUS2_OFF  0x0f38
#define MCUPM_HANDSHAKE_OFFSET  0x0eb4
#define MCUPM_HANDSHAKE_MASK    0xc0002000      /* BIT(30) | BIT(31) | BIT(13) */

/*
 * MTCMOS power-down request/acknowledge pair.
 *
 * LK's clkmgr pdn flow for each zone writes the zone mask to the request word
 * and then spins until the acknowledge word reports the same bits:
 *
 *   0x1c001c34  request
 *   0x1c001c3c  acknowledge
 *
 * The SPM acknowledges exactly the mask it was handed, so keep the pair
 * together instead of reporting a constant: a zone that was never requested
 * still reads as not-acknowledged, and the flow completes only for the zones
 * it actually drove.
 */
#define SPM_PDN_REQ_OFFSET      0x0c34
#define SPM_PDN_ACK_OFFSET      0x0c3c
#define SPM_PDN_MASK            0x00028000      /* BIT(15) | BIT(17) */

/*
 * The SPM driver (mtk-vmm-spm.ko) keeps its own semaphores at 0x69c..0x6b8
 * and a status word at 0xf30; those stay plain storage (the write path below
 * keeps their toggle semantics).  An earlier revision reported the PWR_CON
 * status bits on the whole 0xe10..0xe74 run instead of only on PWR_CON words,
 * which made mtk-vmm-spm.ko believe domains were up and BUG() once the
 * unmodelled hardware behind them was touched.  The PWR_CON word list above
 * is the exact set the driver drives, so nothing else is touched.
 */

/*
 * MTK_PWR_STATE=1 switches the block from the legacy constant-PWR_ACK model
 * (the 0xe10..0xe74 status run) to the real PWR_CON/PWR_STATUS state machine.
 * It is an environment switch so that a regression round can be compared
 * against the previous boot chains without rebuilding; the vendor kernel and
 * the BL2/LK flows must both be verified on the real model first.
 */
static bool mtk_scpsys_real_pwr(void)
{
    static int cached = -1;

    if (cached < 0) {
        cached = getenv("MTK_PWR_STATE") != NULL;
    }
    return cached;
}

/*
 * PWR_STATUS bit per power domain, from the vendor pd_chk table
 * (drivers/clk/mediatek/mtk-pd-chk-mt6895.c pd_pwr_msk[]); the bits are a
 * hardware property of the SPM, not of the consumer.
 */
static const struct {
    uint16_t ctl;
    uint8_t bit;
} mtk_scpsys_pwr_status_map[] = {
    { 0x0e00,  0 },     /* md                  */
    { 0x0e04,  1 },     /* conn                */
    { 0x0e10,  4 },     /* ufs0_shutdown       */
    { 0x0e14,  5 },     /* audio               */
    { 0x0e18,  6 },     /* adsp_top_dormant    */
    { 0x0e1c,  7 },     /* adsp_infra          */
    { 0x0e24,  9 },     /* isp_main            */
    { 0x0e28, 10 },     /* isp_dip1            */
    { 0x0e2c, 11 },     /* isp_ipe             */
    { 0x0e30, 12 },     /* isp_vcore           */
    { 0x0e34, 13 },     /* vde0                */
    { 0x0e38, 14 },     /* vde1                */
    { 0x0e3c, 15 },     /* ven0                */
    { 0x0e40, 16 },     /* ven1                */
    { 0x0e44, 17 },     /* cam_main            */
    { 0x0e48, 18 },     /* cam_mraw            */
    { 0x0e4c, 19 },     /* cam_suba            */
    { 0x0e50, 20 },     /* cam_subb            */
    { 0x0e54, 21 },     /* cam_subc            */
    { 0x0e58, 22 },     /* cam_vcore           */
    { 0x0e5c, 23 },     /* mdp0                */
    { 0x0e60, 24 },     /* mdp1                */
    { 0x0e64, 25 },     /* disp                */
    { 0x0e68, 26 },     /* disp1               */
    { 0x0e6c, 27 },     /* mm_infra            */
    { 0x0e70, 28 },     /* mm_proc_dormant     */
    { 0x0e74, 29 },     /* dp_tx               */
};

/*
 * PWR_STATUS / PWR_STATUS_2ND: per-domain power state.  Both words report
 * the same set on this SoC (the driver requires both to agree).  A domain
 * is powered once its PWR_ON and PWR_ON_2ND bits are driven; the MD domain
 * has no second switch (its MD_OPS flow only sets PWR_ON).
 */
static uint32_t mtk_scpsys_pwr_status(const uint32_t *regs)
{
    uint32_t v = 0;
    size_t i;

    for (i = 0; i < ARRAY_SIZE(mtk_scpsys_pwr_status_map); i++) {
        uint32_t ctl = mtk_scpsys_pwr_status_map[i].ctl;
        uint32_t reg = regs[ctl / 4];
        bool on;

        if (ctl == 0x0e00) {
            on = (reg & PWR_ON_BIT) != 0;
        } else {
            on = (reg & (PWR_ON_BIT | PWR_ON_2ND_BIT)) ==
                 (PWR_ON_BIT | PWR_ON_2ND_BIT);
        }
        if (on) {
            v |= BIT(mtk_scpsys_pwr_status_map[i].bit);
        }
    }
    return v;
}

/*
 * The SPM PWR_CON file is the 0xe00..0xe8 aligned run (md .. dp_tx and the
 * mfg1..mfg12 rail controls).  The gaps and the MCUPM handshake word at
 * 0xeb4 are not domain controls, but the BL2/LK flows read-modify-write the
 * whole run as power-control words, so the state machine covers all of it
 * and the MCUPM case adds its own bits on top.
 */
static bool mtk_scpsys_is_pwr_con(hwaddr o)
{
    return o >= 0x0e00 && o < 0x0ef0 && (o & 3u) == 0;
}

/*
 * PWR_CON read: derive the acknowledge outputs from the request bits, leave
 * the software-control bits as written.  Status bits only report "powered"
 * once both PWR_ON and PWR_ON_2ND have been driven high; the power-off flow
 * clears them one after the other and polls the acknowledge afterwards.
 */
static uint32_t mtk_scpsys_pwr_con_read(uint32_t reg)
{
    uint32_t v = reg & SCPSYS_WRITABLE_MASK;

    if ((v & (PWR_ON_BIT | PWR_ON_2ND_BIT)) ==
        (PWR_ON_BIT | PWR_ON_2ND_BIT)) {
        v |= SCPSYS_STATUS_MASK;
    }
    if (v & SCPSYS_SRAM_PDN_BIT) {
        v |= SCPSYS_SRAM_PDN_ACK;
    }
    if (v & SCPSYS_SRAM_SLP_BIT) {
        v |= SCPSYS_SRAM_SLP_ACK;
    }
    return v;
}

static bool mtk_scpsys_is_mcupm_handshake_reg(hwaddr o)
{
    /*
     * BL2_EXT brings MCUPM up through this same 0x1c001000 window. Its loader
     * writes a command at +0x000, sets bits 2 and 3 of +0xeb4, then spins on
     * "lsr w8, w8, #0x1e; cmp w8, #3" at 0x62f1462c. After the later
     * bit0/bit9 sequence it also spins on bit 13 at 0x62f1466c. The MCUPM
     * core is not emulated, so report the two top bits and bit 13 set, as the
     * generic syscon stub already did for the top bits in the LK path.
     */
    return o == MCUPM_HANDSHAKE_OFFSET;
}

struct MtkScpsysState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t *regs;
    uint32_t size;
};

static bool mtk_scpsys_trace_enabled(void)
{
    static int cached = -1;

    if (cached < 0) {
        cached = getenv("MTK_SCPSYS_TRACE") != NULL;
    }
    return cached;
}

/* MTK_SPIN=1: report a register offset that is being polled forever. */
static void mtk_scpsys_spin_trace(hwaddr off)
{
    static int on = -1;
    static hwaddr last;
    static unsigned long cnt;

    if (on < 0) {
        on = getenv("MTK_SPIN") != NULL;
    }
    if (!on) {
        return;
    }
    if (off == last) {
        cnt++;
        if (cnt == 200 || cnt == 20000 || cnt == 200000) {
            CPUState *cs = qemu_get_cpu(0);
            uint64_t pc = cs ? (uint64_t)CPU_GET_CLASS(cs)->get_pc(cs) : 0;

            fprintf(stderr, "SCPSYS-SPIN off=%#lx x%lu pc=%#" PRIx64 "\n",
                    (unsigned long)off, cnt, pc);
        }
    } else {
        last = off;
        cnt = 1;
    }
}

static uint64_t mtk_scpsys_read(void *opaque, hwaddr offset, unsigned size)
{
    MtkScpsysState *s = MTK_SCPSYS(opaque);
    uint64_t val = 0;
    unsigned int i;

    for (i = 0; i < size; i++) {
        hwaddr o = offset + i;

        if (o < s->size) {
            uint32_t reg = s->regs[o / 4];

            /*
             * The read loop walks byte by byte, so compare the aligned
             * register address: a 4-byte read at 0xe6c visits 0xe6c..0xe6f,
             * and every one of those bytes belongs to the same status
             * register.
             */
            mtk_scpsys_spin_trace(o & ~3u);
            if (mtk_scpsys_real_pwr()) {
                if (mtk_scpsys_is_pwr_con(o & ~3u)) {
                    reg = mtk_scpsys_pwr_con_read(s->regs[(o & ~3u) / 4]);
                } else if ((o & ~3u) == SCPSYS_PWR_STATUS_OFF ||
                           (o & ~3u) == SCPSYS_PWR_STATUS2_OFF) {
                    /*
                     * PWR_STATUS / PWR_STATUS_2ND are read-only outputs:
                     * derive them from the PWR_CON state.
                     */
                    reg = mtk_scpsys_pwr_status(s->regs);
                }
            } else if ((o & ~3u) >= 0x0e10 && (o & ~3u) <= 0x0e74) {
                /* Legacy constant PWR_ACK, kept for the pre-MTK_PWR_STATE model. */
                reg |= SCPSYS_STATUS_MASK;
            }
            if (mtk_scpsys_is_mcupm_handshake_reg(o & ~3u)) {
                reg |= MCUPM_HANDSHAKE_MASK;
            }
            if (getenv("MTK_SCP0") && (o & ~3u) >= 0xc00 && (o & ~3u) <= 0xc80) {
                static int n;

                if (n < 40) {
                    n++;
                    fprintf(stderr, "SCP0 R off=%#lx reg=%#x req=%#x\n",
                            (unsigned long)(o & ~3u), reg,
                            s->regs[SPM_PDN_REQ_OFFSET / 4]);
                }
            }
            if ((o & ~3u) == SPM_PDN_ACK_OFFSET) {
                /* Acknowledge the zone mask the flow actually requested. */
                reg |= s->regs[SPM_PDN_REQ_OFFSET / 4] & SPM_PDN_MASK;
            }
            if (mtk_scpsys_trace_enabled()) {
                static uint64_t counts[0x1000 / 4];
                static uint64_t total;
                static int dumps;
                static int first;

                if (first < 60) {
                    first++;
                    fprintf(stderr, "SCPSYS-TRACE first R off=%#lx size=%u val=%#"
                            PRIx64 "\n", (unsigned long)(o & ~3u), size,
                            (uint64_t)reg);
                }

                counts[(o & ~3u) / 4]++;
                total++;
                if (total >= (uint64_t)(dumps + 1) * 200 && dumps < 10) {
                    int a;

                    dumps++;
                    fprintf(stderr, "SCPSYS-TRACE read dump %d (total=%" PRIu64
                            "):\n", dumps, total);
                    fprintf(stderr, "  hot:");
                    for (a = 0; a < 0x400; a++) {
                        if (counts[a] >= 6) {
                            fprintf(stderr, " +%#x=%" PRIu64, a * 4, counts[a]);
                        }
                    }
                    fprintf(stderr, "\n");
                    memset(counts, 0, sizeof(counts));
                }
            }
            val |= (uint64_t)((reg >> (8 * (o % 4))) & 0xff) << (8 * i);
        }
    }
    trace_mtk_scpsys_read(offset, val);
    return val;
}

static void mtk_scpsys_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    MtkScpsysState *s = MTK_SCPSYS(opaque);
    unsigned int i;

    for (i = 0; i < size; i++) {
        hwaddr o = offset + i;

        if (o < s->size) {
            uint32_t *reg = &s->regs[o / 4];
            unsigned shift = 8 * (o % 4);
            uint32_t mask = 0xffu << shift;

            if ((o & ~3u) >= 0x069c && (o & ~3u) <= 0x06b8 &&
                !getenv("MTK_NO_SEMTOGGLE")) {
                /*
                 * SPM hardware semaphores: writing 1 toggles ownership
                 * (acquire when free, release when held), so a release
                 * write must clear the bit. Plain storage left it set and
                 * mtk-vmm-spm release_hw_semaphore() hit BUG().
                 */
                *reg ^= (((uint32_t)value >> (8 * i)) & 0xff) << shift;
                continue;
            }
            *reg = (*reg & ~mask) |
                   ((((uint32_t)value >> (8 * i)) & 0xff) << shift);
            if (getenv("MTK_SCP0") && (o & ~3u) >= 0xc00 && (o & ~3u) <= 0xc80) {
                static int nw;

                if (nw < 40) {
                    nw++;
                    fprintf(stderr, "SCP0 W off=%#lx val=%#x\n",
                            (unsigned long)(o & ~3u), *reg);
                }
            }
        }
    }
    if (mtk_scpsys_real_pwr() && mtk_scpsys_is_pwr_con(offset & ~3u)) {
        /* Acknowledge/status bits are outputs of the block, not writable. */
        s->regs[(offset & ~3u) / 4] &= SCPSYS_WRITABLE_MASK;
    }
    if (mtk_scpsys_trace_enabled()) {
        static uint64_t wcounts[0x1000 / 4];
        static uint64_t wtotal;
        static int wdumps;

        wcounts[(offset & ~3u) / 4]++;
        wtotal++;
        {
            static int wfirst;

            if (wfirst < 40) {
                wfirst++;
                fprintf(stderr, "SCPSYS-TRACE first W off=%#lx size=%u val=%#"
                        PRIx64 "\n", (unsigned long)(offset & ~3u), size, value);
            }
        }
        if (wtotal >= (uint64_t)(wdumps + 1) * 5000 && wdumps < 6) {
            int a, b;

            wdumps++;
            fprintf(stderr, "SCPSYS-TRACE write dump %d (total=%" PRIu64 "):\n",
                    wdumps, wtotal);
            for (b = 0; b < 10; b++) {
                unsigned best = 0;
                uint64_t bestn = 0;

                for (a = 0; a < 0x400; a++) {
                    if (wcounts[a] > bestn) {
                        bestn = wcounts[a];
                        best = a;
                    }
                }
                if (!bestn) {
                    break;
                }
                fprintf(stderr, "  W +%#05x: %" PRIu64 "\n", best * 4, bestn);
                wcounts[best] = 0;
            }
        }
    }
    trace_mtk_scpsys_write(offset, value);
}

static const MemoryRegionOps mtk_scpsys_ops = {
    .read = mtk_scpsys_read,
    .write = mtk_scpsys_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void mtk_scpsys_realize(DeviceState *dev, Error **errp)
{
    MtkScpsysState *s = MTK_SCPSYS(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &mtk_scpsys_ops, s,
                          TYPE_MTK_SCPSYS, s->size);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    s->regs = g_new0(uint32_t, s->size / 4);
}

static void mtk_scpsys_finalize(Object *obj)
{
    MtkScpsysState *s = MTK_SCPSYS(obj);

    g_free(s->regs);
}

static const Property mtk_scpsys_props[] = {
    DEFINE_PROP_UINT32("size", MtkScpsysState, size, 0x1000),
};

static void mtk_scpsys_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = mtk_scpsys_realize;
    device_class_set_props(dc, mtk_scpsys_props);
    dc->desc = "MediaTek SCPSYS power-domain controller";
}

static const TypeInfo mtk_scpsys_info = {
    .name = TYPE_MTK_SCPSYS,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MtkScpsysState),
    .instance_finalize = mtk_scpsys_finalize,
    .class_init = mtk_scpsys_class_init,
};

static void mtk_scpsys_register_types(void)
{
    type_register_static(&mtk_scpsys_info);
}

type_init(mtk_scpsys_register_types)
