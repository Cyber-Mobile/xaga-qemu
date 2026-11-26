/*
 * MediaTek MT6895 LVTS (Low Voltage Thermal Sensor) and TIA ADC model
 *
 * Models:
 *   - MMIO 0 (0x10315000, size 0x1000): LVTS Domain 0 (5 TCs at stride 0x100,
 *     TC3..7 with hardware device IDs 0x85..0x89 at +0x4c).
 *   - MMIO 1 (0x10316000, size 0x1000): LVTS Domain 1 (3 TCs at stride 0x100,
 *     TC0..2 with hardware device IDs 0x81..0x83 at +0x4c).
 *   - MMIO 2 (0x1c805000, size 0x1000): Thermal Interface ADC (TIA), reporting
 *     nominal valid room-temperature NTC samples (0x0000c000: data_valid |
 *     rc_sel=0 | raw=0x4000) across 0x0cc..0x0f0 while preserving RW storage
 *     for the remaining control registers (e.g. GPS TIA ctrl at +0x28/+0x30).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/core/sysbus.h"
#include "hw/misc/mtk_lvts.h"
#include "system/memory.h"

#define LVTS_D0_NUM_TC      5
#define LVTS_D1_NUM_TC      3
#define LVTS_TC_STRIDE      0x100
#define LVTS_REG_WORDS      (MTK_LVTS_WIN_SIZE / 4)

#define LVTS_REG_ID         0x4c
#define LVTS_REG_CONFIG0    0x50
#define LVTS_REG_MSR0       0x90
#define LVTS_REG_MSR3       0x9c
#define LVTS_REG_RDATA0     0xb0

#define LVTS_CMD_START_BIT  BIT(24)
#define LVTS_SENSING_STATUS BIT(25)
#define LVTS_MSR_NOMINAL    0x00014000u  /* valid bit16 | raw 0x4000 */
#define LVTS_RDATA_NOMINAL  0x00004000u  /* nominal 14-bit COUNT_RC_N */

#define TIA_DATA_START      0x0cc
#define TIA_DATA_END        0x0f4        /* 10 channels: 0x0cc..0x0f0 */
#define TIA_ADC_NOMINAL     0x0000c000u  /* valid bit15 | rc_sel=0 | 0x4000 */

struct MtkLvtsState {
    SysBusDevice parent_obj;

    MemoryRegion iomem_d0;
    MemoryRegion iomem_d1;
    MemoryRegion iomem_tia;

    uint32_t d0_regs[LVTS_REG_WORDS];
    uint32_t d1_regs[LVTS_REG_WORDS];
    uint32_t d0_rdata[LVTS_D0_NUM_TC];
    uint32_t d1_rdata[LVTS_D1_NUM_TC];
    uint32_t tia_regs[LVTS_REG_WORDS];
};

static uint32_t lvts_tc_device_id(unsigned domain, unsigned tc)
{
    return (domain == 0) ? (0x85u + tc) : (0x81u + tc);
}

static uint32_t lvts_domain_read_word(MtkLvtsState *s, unsigned domain,
                                      hwaddr off)
{
    unsigned num_tc = (domain == 0) ? LVTS_D0_NUM_TC : LVTS_D1_NUM_TC;
    uint32_t *regs = (domain == 0) ? s->d0_regs : s->d1_regs;
    uint32_t *rdata = (domain == 0) ? s->d0_rdata : s->d1_rdata;
    unsigned tc = (unsigned)(off / LVTS_TC_STRIDE);
    unsigned reg = (unsigned)(off % LVTS_TC_STRIDE);

    if (tc < num_tc) {
        if (reg == LVTS_REG_ID) {
            return lvts_tc_device_id(domain, tc);
        }
        if (reg == LVTS_REG_RDATA0) {
            return rdata[tc];
        }
        if (reg >= LVTS_REG_MSR0 && reg <= LVTS_REG_MSR3) {
            return LVTS_MSR_NOMINAL;
        }
    }
    return regs[(off >> 2) & (LVTS_REG_WORDS - 1)];
}

static void lvts_domain_write_word(MtkLvtsState *s, unsigned domain,
                                   hwaddr off, uint32_t val)
{
    unsigned num_tc = (domain == 0) ? LVTS_D0_NUM_TC : LVTS_D1_NUM_TC;
    uint32_t *regs = (domain == 0) ? s->d0_regs : s->d1_regs;
    uint32_t *rdata = (domain == 0) ? s->d0_rdata : s->d1_rdata;
    unsigned tc = (unsigned)(off / LVTS_TC_STRIDE);
    unsigned reg = (unsigned)(off % LVTS_TC_STRIDE);

    if (tc < num_tc && reg == LVTS_REG_CONFIG0) {
        if ((val & 0xffff0000u) == 0xc5020000u) {
            rdata[tc] = lvts_tc_device_id(domain, tc);
        } else if ((val & 0xff000000u) == 0xc1000000u) {
            rdata[tc] = LVTS_RDATA_NOMINAL;
        }
        val &= ~(LVTS_CMD_START_BIT | LVTS_SENSING_STATUS);
    }
    regs[(off >> 2) & (LVTS_REG_WORDS - 1)] = val;
}

static uint64_t lvts_domain_read(MtkLvtsState *s, unsigned domain,
                                 hwaddr offset, unsigned size)
{
    uint64_t out = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        hwaddr o = offset + i;
        if (o < MTK_LVTS_WIN_SIZE) {
            uint32_t w = lvts_domain_read_word(s, domain, o & ~3ULL);
            out |= (uint64_t)((w >> (8 * (o & 3))) & 0xffu) << (8 * i);
        }
    }
    return out;
}

static void lvts_domain_write(MtkLvtsState *s, unsigned domain,
                              hwaddr offset, uint64_t value, unsigned size)
{
    unsigned i;

    for (i = 0; i < size; i++) {
        hwaddr o = offset + i;
        if (o < MTK_LVTS_WIN_SIZE) {
            hwaddr aligned = o & ~3ULL;
            unsigned shift = 8 * (o & 3);
            uint32_t mask = 0xffu << shift;
            uint32_t cur = lvts_domain_read_word(s, domain, aligned);
            uint32_t next = (cur & ~mask) |
                            ((((uint32_t)value >> (8 * i)) & 0xffu) << shift);
            lvts_domain_write_word(s, domain, aligned, next);
        }
    }
}

static uint64_t mtk_lvts_d0_read(void *opaque, hwaddr off, unsigned size)
{
    return lvts_domain_read(MTK_LVTS(opaque), 0, off, size);
}

static void mtk_lvts_d0_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    lvts_domain_write(MTK_LVTS(opaque), 0, off, val, size);
}

static uint64_t mtk_lvts_d1_read(void *opaque, hwaddr off, unsigned size)
{
    return lvts_domain_read(MTK_LVTS(opaque), 1, off, size);
}

static void mtk_lvts_d1_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    lvts_domain_write(MTK_LVTS(opaque), 1, off, val, size);
}

static uint32_t mtk_tia_read_word(MtkLvtsState *s, hwaddr off)
{
    if (off >= TIA_DATA_START && off < TIA_DATA_END) {
        return TIA_ADC_NOMINAL;
    }
    return s->tia_regs[(off >> 2) & (LVTS_REG_WORDS - 1)];
}

static uint64_t mtk_tia_read(void *opaque, hwaddr offset, unsigned size)
{
    MtkLvtsState *s = MTK_LVTS(opaque);
    uint64_t out = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        hwaddr o = offset + i;
        if (o < MTK_TIA_WIN_SIZE) {
            uint32_t w = mtk_tia_read_word(s, o & ~3ULL);
            out |= (uint64_t)((w >> (8 * (o & 3))) & 0xffu) << (8 * i);
        }
    }
    return out;
}

static void mtk_tia_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    MtkLvtsState *s = MTK_LVTS(opaque);
    unsigned i;

    for (i = 0; i < size; i++) {
        hwaddr o = offset + i;
        if (o < MTK_TIA_WIN_SIZE) {
            uint32_t *reg = &s->tia_regs[(o >> 2) & (LVTS_REG_WORDS - 1)];
            unsigned shift = 8 * (o & 3);
            uint32_t mask = 0xffu << shift;
            *reg = (*reg & ~mask) |
                   ((((uint32_t)value >> (8 * i)) & 0xffu) << shift);
        }
    }
}

static const MemoryRegionOps mtk_lvts_d0_ops = {
    .read = mtk_lvts_d0_read,
    .write = mtk_lvts_d0_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static const MemoryRegionOps mtk_lvts_d1_ops = {
    .read = mtk_lvts_d1_read,
    .write = mtk_lvts_d1_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static const MemoryRegionOps mtk_tia_ops = {
    .read = mtk_tia_read,
    .write = mtk_tia_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static void mtk_lvts_reset(DeviceState *dev)
{
    MtkLvtsState *s = MTK_LVTS(dev);
    unsigned i;

    memset(s->d0_regs, 0, sizeof(s->d0_regs));
    memset(s->d1_regs, 0, sizeof(s->d1_regs));
    memset(s->tia_regs, 0, sizeof(s->tia_regs));
    for (i = 0; i < LVTS_D0_NUM_TC; i++) {
        s->d0_rdata[i] = LVTS_RDATA_NOMINAL;
    }
    for (i = 0; i < LVTS_D1_NUM_TC; i++) {
        s->d1_rdata[i] = LVTS_RDATA_NOMINAL;
    }
}

static void mtk_lvts_realize(DeviceState *dev, Error **errp)
{
    MtkLvtsState *s = MTK_LVTS(dev);

    (void)errp;
    memory_region_init_io(&s->iomem_d0, OBJECT(dev), &mtk_lvts_d0_ops, s,
                          TYPE_MTK_LVTS ".domain0", MTK_LVTS_WIN_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem_d0);

    memory_region_init_io(&s->iomem_d1, OBJECT(dev), &mtk_lvts_d1_ops, s,
                          TYPE_MTK_LVTS ".domain1", MTK_LVTS_WIN_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem_d1);

    memory_region_init_io(&s->iomem_tia, OBJECT(dev), &mtk_tia_ops, s,
                          TYPE_MTK_LVTS ".tia", MTK_TIA_WIN_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem_tia);
}

static void mtk_lvts_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    (void)data;
    dc->realize = mtk_lvts_realize;
    dc->desc = "MediaTek MT6895 LVTS thermal controller and TIA ADC";
    device_class_set_legacy_reset(dc, mtk_lvts_reset);
}

static const TypeInfo mtk_lvts_info = {
    .name = TYPE_MTK_LVTS,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MtkLvtsState),
    .class_init = mtk_lvts_class_init,
};

static void mtk_lvts_register_types(void)
{
    type_register_static(&mtk_lvts_info);
}

type_init(mtk_lvts_register_types)
