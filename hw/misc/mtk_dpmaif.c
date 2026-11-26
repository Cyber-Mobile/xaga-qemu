/*
 * MediaTek MT6895 DPMAIF (Data Plane Modem AP Interface v3) model
 *
 * Models the DPMAIF v3 control/status register semantics required by
 * ccci_dpmaif.ko:
 *   - MMIO 0 (0x10014000, size 0x1000): AO_UL / AO_DL / AO_MISC
 *     +0x080 UL_INT_MASK, +0x084 UL_INT_MASK_CLR, +0x088 UL_INT_MASK_SET
 *     +0x090 DL_INT_MASK, +0x094 DL_INT_MASK_CLR, +0x098 DL_INT_MASK_SET
 *   - MMIO 1 (0x1022d000, size 0x1000): PD_UL (+0x000), PD_DL (+0x100),
 *     PD_MISC (+0x400)
 *     +0x088 UL idle (BIT(11) idle)
 *     +0x0b0..+0x0bc UL add wcnt (BIT(31) self-clearing)
 *     +0x100/+0x104/+0x120/+0x124 DL init/cnt (BIT(31) self-clearing)
 *     +0x1b4 DL idle (BIT(23) idle)
 *     +0x400/+0x450/+0x460 W1C interrupt status
 *     +0x490 HW_INIT_DONE / DL_RESTORE status (BIT(0) self-clearing)
 *     +0x494 SRAM_INIT (BIT(0) self-clearing)
 *     +0x498 DL_RESTORE_SET (sets written bits in +0x490)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/core/sysbus.h"
#include "hw/misc/mtk_dpmaif.h"
#include "system/memory.h"

#define DPMAIF_REG_WORDS            (MTK_DPMAIF_WIN_SIZE / 4)

/* AO register offsets */
#define AO_UL_INT_MASK              0x080
#define AO_UL_INT_MASK_CLR          0x084
#define AO_UL_INT_MASK_SET          0x088
#define AO_DL_INT_MASK              0x090
#define AO_DL_INT_MASK_CLR          0x094
#define AO_DL_INT_MASK_SET          0x098

/* PD register offsets */
#define PD_UL_IDLE_CHECK            0x088
#define PD_UL_ADD_WCNT_START        0x0b0
#define PD_UL_ADD_WCNT_END          0x0bc
#define PD_DL_BAT_INIT_DONE         0x100
#define PD_DL_ADD_BAT_CNT           0x104
#define PD_DL_PIT_INIT_DONE         0x120
#define PD_DL_ADD_PIT_CNT           0x124
#define PD_DL_IDLE_CHECK            0x1b4
#define PD_MISC_AP_L2TISAR0         0x400
#define PD_MISC_AP_IP_BUSY          0x450
#define PD_MISC_CG_EN_STATUS        0x460
#define PD_MISC_HW_INIT_DONE        0x490
#define PD_MISC_SRAM_INIT           0x494
#define PD_MISC_DL_RESTORE_SET      0x498

struct MtkDpmaifState {
    SysBusDevice parent_obj;

    MemoryRegion iomem_ao;
    MemoryRegion iomem_pd;

    uint32_t ao_regs[DPMAIF_REG_WORDS];
    uint32_t pd_regs[DPMAIF_REG_WORDS];
};

static uint32_t dpmaif_ao_read_word(MtkDpmaifState *s, hwaddr off)
{
    return s->ao_regs[(off >> 2) & (DPMAIF_REG_WORDS - 1)];
}

static void dpmaif_ao_write_word(MtkDpmaifState *s, hwaddr off,
                                uint32_t val, uint32_t mask)
{
    uint32_t *reg = &s->ao_regs[off >> 2];

    switch (off) {
    case AO_UL_INT_MASK_CLR:
        s->ao_regs[AO_UL_INT_MASK >> 2] &= ~val;
        break;
    case AO_UL_INT_MASK_SET:
        s->ao_regs[AO_UL_INT_MASK >> 2] |= val;
        break;
    case AO_DL_INT_MASK_CLR:
        s->ao_regs[AO_DL_INT_MASK >> 2] &= ~val;
        break;
    case AO_DL_INT_MASK_SET:
        s->ao_regs[AO_DL_INT_MASK >> 2] |= val;
        break;
    }
    *reg = (*reg & ~mask) | val;
}

static uint32_t dpmaif_pd_read_word(MtkDpmaifState *s, hwaddr off)
{
    uint32_t val = s->pd_regs[(off >> 2) & (DPMAIF_REG_WORDS - 1)];

    if (off == PD_UL_IDLE_CHECK) {
        val |= BIT(11);
    } else if (off == PD_DL_IDLE_CHECK) {
        val |= BIT(23);
    }
    return val;
}

static void dpmaif_pd_write_word(MtkDpmaifState *s, hwaddr off,
                                uint32_t val, uint32_t mask)
{
    uint32_t *reg = &s->pd_regs[off >> 2];
    uint32_t next = (*reg & ~mask) | val;

    if ((off >= PD_UL_ADD_WCNT_START && off <= PD_UL_ADD_WCNT_END) ||
        off == PD_DL_BAT_INIT_DONE || off == PD_DL_ADD_BAT_CNT ||
        off == PD_DL_PIT_INIT_DONE || off == PD_DL_ADD_PIT_CNT) {
        *reg = next & ~(uint32_t)BIT(31);
        return;
    }

    switch (off) {
    case PD_MISC_AP_L2TISAR0:
    case PD_MISC_AP_IP_BUSY:
    case PD_MISC_CG_EN_STATUS:
        *reg &= ~val;
        break;
    case PD_MISC_HW_INIT_DONE:
    case PD_MISC_SRAM_INIT:
        *reg = next & ~1u;
        break;
    case PD_MISC_DL_RESTORE_SET:
        *reg = next;
        s->pd_regs[PD_MISC_HW_INIT_DONE >> 2] |= val;
        break;
    default:
        *reg = next;
        break;
    }
}

static uint64_t mtk_dpmaif_ao_read(void *opaque, hwaddr offset, unsigned size)
{
    MtkDpmaifState *s = MTK_DPMAIF(opaque);
    uint64_t out = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        hwaddr o = offset + i;
        if (o < MTK_DPMAIF_WIN_SIZE) {
            uint32_t w = dpmaif_ao_read_word(s, o & ~3ULL);
            out |= (uint64_t)((w >> (8 * (o & 3))) & 0xffu) << (8 * i);
        }
    }
    return out;
}

static void mtk_dpmaif_ao_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    MtkDpmaifState *s = MTK_DPMAIF(opaque);
    hwaddr word_off = offset & ~3ULL;
    unsigned shift = (offset & 3) * 8;
    uint32_t mask = MAKE_64BIT_MASK(shift, size * 8);

    dpmaif_ao_write_word(s, word_off, (value << shift) & mask, mask);
}

static uint64_t mtk_dpmaif_pd_read(void *opaque, hwaddr offset, unsigned size)
{
    MtkDpmaifState *s = MTK_DPMAIF(opaque);
    uint64_t out = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        hwaddr o = offset + i;
        if (o < MTK_DPMAIF_WIN_SIZE) {
            uint32_t w = dpmaif_pd_read_word(s, o & ~3ULL);
            out |= (uint64_t)((w >> (8 * (o & 3))) & 0xffu) << (8 * i);
        }
    }
    return out;
}

static void mtk_dpmaif_pd_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    MtkDpmaifState *s = MTK_DPMAIF(opaque);
    hwaddr word_off = offset & ~3ULL;
    unsigned shift = (offset & 3) * 8;
    uint32_t mask = MAKE_64BIT_MASK(shift, size * 8);

    dpmaif_pd_write_word(s, word_off, (value << shift) & mask, mask);
}

/* The driver's register dump reads pairs of 32-bit registers with LDP. */
static const MemoryRegionOps mtk_dpmaif_ao_ops = {
    .read = mtk_dpmaif_ao_read,
    .write = mtk_dpmaif_ao_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static const MemoryRegionOps mtk_dpmaif_pd_ops = {
    .read = mtk_dpmaif_pd_read,
    .write = mtk_dpmaif_pd_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static void mtk_dpmaif_reset(DeviceState *dev)
{
    MtkDpmaifState *s = MTK_DPMAIF(dev);

    memset(s->ao_regs, 0, sizeof(s->ao_regs));
    memset(s->pd_regs, 0, sizeof(s->pd_regs));
    s->ao_regs[AO_UL_INT_MASK >> 2] = 0xffffffffu;
    s->ao_regs[AO_DL_INT_MASK >> 2] = 0xffffffffu;
}

static void mtk_dpmaif_realize(DeviceState *dev, Error **errp)
{
    MtkDpmaifState *s = MTK_DPMAIF(dev);

    (void)errp;
    memory_region_init_io(&s->iomem_ao, OBJECT(dev), &mtk_dpmaif_ao_ops, s,
                          TYPE_MTK_DPMAIF ".ao", MTK_DPMAIF_WIN_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem_ao);

    memory_region_init_io(&s->iomem_pd, OBJECT(dev), &mtk_dpmaif_pd_ops, s,
                          TYPE_MTK_DPMAIF ".pd", MTK_DPMAIF_WIN_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem_pd);
}

static void mtk_dpmaif_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    (void)data;
    dc->realize = mtk_dpmaif_realize;
    dc->desc = "MediaTek MT6895 DPMAIF v3 host interface";
    device_class_set_legacy_reset(dc, mtk_dpmaif_reset);
}

static const TypeInfo mtk_dpmaif_info = {
    .name = TYPE_MTK_DPMAIF,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MtkDpmaifState),
    .class_init = mtk_dpmaif_class_init,
};

static void mtk_dpmaif_register_types(void)
{
    type_register_static(&mtk_dpmaif_info);
}

type_init(mtk_dpmaif_register_types)
