/*
 * MediaTek MT6895 cpufreq-hw (CSRAM DVFS performance controller) model
 *
 * Models the three CPU DVFS domains in CSRAM (overlaying SRAM0 at 0x0011bc10):
 *   - Domain 0 (0x0011bc10, size 0x120): 4x Cortex-A55 (cpu0..3)
 *   - Domain 1 (0x0011bd30, size 0x120): 3x Cortex-A78 (cpu4..6)
 *   - Domain 2 (0x0011be50, size 0x120): 1x Cortex-A78 Prime (cpu7)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/core/sysbus.h"
#include "hw/misc/mtk_cpufreq_hw.h"
#include "system/memory.h"

#define CPUFREQ_DOM_WORDS       (MTK_CPUFREQ_HW_DOM_SIZE / 4)
#define CPUFREQ_LUT_ENTRIES     32
#define CPUFREQ_OPP_COUNT       8

#define REG_FREQ_LUT_START      0x000
#define REG_FREQ_LUT_END        0x080
#define REG_FREQ_ENABLE         0x084
#define REG_FREQ_PERF_STATE     0x088
#define REG_FREQ_HW_STATE       0x08c
#define REG_EM_POWER_START      0x090
#define REG_EM_POWER_END        0x110
#define REG_FREQ_LATENCY        0x114

#define CPUFREQ_HW_ENABLED      0x3u    /* bit 0: HW enabled, bit 1: SVS enabled */
#define CPUFREQ_LATENCY_US      1000u

/*
 * The vendor driver walks the LUT from highest to lowest frequency and stops
 * at the first repeated entry. The per-domain frequencies and power estimates
 * below are experimental values pending measured MT6895 OPP tables.
 */
static const uint32_t mtk_cpufreq_lut[MTK_CPUFREQ_HW_DOMAINS][CPUFREQ_OPP_COUNT] = {
    { 2000, 1701, 1500, 1300, 1100, 900, 700, 500 },
    { 2850, 2500, 2171, 1900, 1600, 1300, 1000, 700 },
    { 2850, 2500, 2171, 1900, 1600, 1300, 1000, 700 },
};

static const uint32_t mtk_cpufreq_power_uw[MTK_CPUFREQ_HW_DOMAINS][CPUFREQ_OPP_COUNT] = {
    { 520000, 400000, 310000, 230000, 170000, 120000, 80000, 50000 },
    { 1850000, 1400000, 1050000, 800000, 580000, 400000, 260000, 150000 },
    { 1980000, 1500000, 1120000, 860000, 620000, 430000, 280000, 160000 },
};

struct MtkCpufreqHwState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion eas_limits;
    uint32_t regs[MTK_CPUFREQ_HW_DOMAINS][CPUFREQ_DOM_WORDS];
};

static uint32_t mtk_cpufreq_read_word(MtkCpufreqHwState *s, hwaddr off)
{
    unsigned dom = (unsigned)(off / MTK_CPUFREQ_HW_DOM_SIZE);
    unsigned doff = (unsigned)(off % MTK_CPUFREQ_HW_DOM_SIZE);

    if (dom >= MTK_CPUFREQ_HW_DOMAINS) {
        return 0;
    }
    return s->regs[dom][doff >> 2];
}

static void mtk_cpufreq_write_word(MtkCpufreqHwState *s, hwaddr off,
                                   uint32_t val)
{
    unsigned dom = (unsigned)(off / MTK_CPUFREQ_HW_DOM_SIZE);
    unsigned doff = (unsigned)(off % MTK_CPUFREQ_HW_DOM_SIZE);

    if (dom >= MTK_CPUFREQ_HW_DOMAINS) {
        return;
    }

    if (doff < REG_FREQ_LUT_END || doff == REG_FREQ_HW_STATE ||
        (doff >= REG_EM_POWER_START && doff < REG_EM_POWER_END) ||
        doff == REG_FREQ_LATENCY) {
        return;
    }

    if (doff == REG_FREQ_ENABLE) {
        s->regs[dom][REG_FREQ_ENABLE >> 2] = val;
        s->regs[dom][REG_FREQ_HW_STATE >> 2] = (val & 1u) ? CPUFREQ_HW_ENABLED : 0u;
        return;
    }

    if (doff == REG_FREQ_PERF_STATE) {
        s->regs[dom][REG_FREQ_PERF_STATE >> 2] = val & 0x1fu;
        return;
    }

    s->regs[dom][doff >> 2] = val;
}

static uint64_t mtk_cpufreq_hw_read(void *opaque, hwaddr offset, unsigned size)
{
    MtkCpufreqHwState *s = MTK_CPUFREQ_HW(opaque);
    uint64_t out = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        hwaddr o = offset + i;
        if (o < MTK_CPUFREQ_HW_SIZE) {
            uint32_t w = mtk_cpufreq_read_word(s, o & ~3ULL);
            out |= (uint64_t)((w >> (8 * (o & 3))) & 0xffu) << (8 * i);
        }
    }
    return out;
}

static void mtk_cpufreq_hw_write(void *opaque, hwaddr offset, uint64_t value,
                                 unsigned size)
{
    MtkCpufreqHwState *s = MTK_CPUFREQ_HW(opaque);
    hwaddr word_off = offset & ~3ULL;
    uint32_t cur, mask, shift, new_val;

    if (word_off >= MTK_CPUFREQ_HW_SIZE) {
        return;
    }

    if (size == 4 && (offset & 3) == 0) {
        mtk_cpufreq_write_word(s, word_off, (uint32_t)value);
        return;
    }

    shift = (unsigned)(offset & 3) * 8;
    mask = (size >= 4) ? 0xffffffffu : ((1u << (size * 8)) - 1u);
    cur = mtk_cpufreq_read_word(s, word_off);
    new_val = (cur & ~(mask << shift)) | (((uint32_t)value & mask) << shift);
    mtk_cpufreq_write_word(s, word_off, new_val);
}

static const MemoryRegionOps mtk_cpufreq_hw_ops = {
    .read = mtk_cpufreq_hw_read,
    .write = mtk_cpufreq_hw_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static void mtk_cpufreq_hw_reset(DeviceState *dev)
{
    MtkCpufreqHwState *s = MTK_CPUFREQ_HW(dev);
    unsigned dom, i;

    /* The reset performance state is the highest OPP, at LUT index zero. */
    memset(s->regs, 0, sizeof(s->regs));
    /* The EAS limit starts at the highest permitted OPP in each domain. */
    memset(memory_region_get_ram_ptr(&s->eas_limits), 0,
           MTK_CPUFREQ_EAS_LIMIT_SIZE);
    for (dom = 0; dom < MTK_CPUFREQ_HW_DOMAINS; dom++) {
        for (i = 0; i < CPUFREQ_LUT_ENTRIES; i++) {
            unsigned idx = (i < CPUFREQ_OPP_COUNT) ? i : (CPUFREQ_OPP_COUNT - 1);
            s->regs[dom][(REG_FREQ_LUT_START >> 2) + i] = mtk_cpufreq_lut[dom][idx];
            s->regs[dom][(REG_EM_POWER_START >> 2) + i] = mtk_cpufreq_power_uw[dom][idx];
        }
        s->regs[dom][REG_FREQ_HW_STATE >> 2] = CPUFREQ_HW_ENABLED;
        s->regs[dom][REG_FREQ_LATENCY >> 2] = CPUFREQ_LATENCY_US;
    }
}

static void mtk_cpufreq_hw_realize(DeviceState *dev, Error **errp)
{
    MtkCpufreqHwState *s = MTK_CPUFREQ_HW(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &mtk_cpufreq_hw_ops, s,
                          TYPE_MTK_CPUFREQ_HW, MTK_CPUFREQ_HW_SIZE);
    /*
     * eas_info points at four CSRAM thermal limit indices (L/BL/B/CCI).
     * Keep this shared state separate from the BootROM's preloader copy in
     * SRAM0. The scheduler reads it directly to index the energy model.
     */
    if (!memory_region_init_ram(&s->eas_limits, OBJECT(dev),
                                TYPE_MTK_CPUFREQ_HW ".eas-limits",
                                MTK_CPUFREQ_EAS_LIMIT_SIZE, errp)) {
        return;
    }
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->eas_limits);
}

static void mtk_cpufreq_hw_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    (void)data;
    dc->realize = mtk_cpufreq_hw_realize;
    dc->desc = "MediaTek MT6895 cpufreq-hw CSRAM controller";
    device_class_set_legacy_reset(dc, mtk_cpufreq_hw_reset);
}

static const TypeInfo mtk_cpufreq_hw_info = {
    .name = TYPE_MTK_CPUFREQ_HW,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MtkCpufreqHwState),
    .class_init = mtk_cpufreq_hw_class_init,
};

static void mtk_cpufreq_hw_register_types(void)
{
    type_register_static(&mtk_cpufreq_hw_info);
}

type_init(mtk_cpufreq_hw_register_types)
