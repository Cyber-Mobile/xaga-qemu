/*
 * MediaTek M4U / IOMMU register window (one 4 KiB bank) for the MT6895 model.
 *
 * Semantics follow the vendor mtk_iommu.ko shipped with the firmware
 * (vendor module analysis, mtk_iommu_tlb_flush_all @0x1660)
 * and upstream drivers/iommu/mtk_iommu.c:
 *   - INV_SEL (0x2c gen1 / 0x38 gen2) and INVLD_START/END (0x24/0x28) are
 *     plain registers;
 *   - writing F_MMU_INV_RANGE (bit0) or F_ALL_INVLD (bit1) to
 *     REG_MMU_INVALIDATE (0x20) starts a TLB invalidation; the model has no
 *     TLB, so the operation completes at once and REG_MMU_CPE_DONE (0x12c)
 *     becomes non-zero.  The driver polls CPE_DONE != 0 and then writes 0
 *     to it (vendor insn 0x1920), which clears it;
 *   - REG_MMU_INT_CONTROL0 (0x120) bit12 F_INT_CLR_BIT is a write-1 pulse
 *     that clears the latched fault status; no faults are ever latched here,
 *     so the fault status/address registers (0x134..0x15c) read 0;
 *   - every other register (CTRL_REG 0x110, PT base, larb/port config, ...)
 *     keeps what software wrote.
 * MTK_IOMMU_LOG=1 traces invalidations on stderr.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qom/object.h"

#define TYPE_MTK_IOMMU "mtk-iommu"
OBJECT_DECLARE_SIMPLE_TYPE(MtkIommuState, MTK_IOMMU)

#define MTK_IOMMU_SIZE           0x1000
#define REG_MMU_INVALIDATE       0x020
#define   F_MMU_INV_RANGE        (1u << 0)
#define   F_ALL_INVLD            (1u << 1)
#define REG_MMU_INT_CONTROL0     0x120
#define   F_INT_CLR_BIT          (1u << 12)
#define REG_MMU_CPE_DONE         0x12c
#define REG_MMU_FAULT_ST_FIRST   0x134
#define REG_MMU_FAULT_ST_LAST    0x15c

struct MtkIommuState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[MTK_IOMMU_SIZE / 4];
    uint32_t n_inval;
    bool log;
};

static uint64_t mtk_iommu_read(void *opaque, hwaddr addr, unsigned size)
{
    MtkIommuState *s = opaque;

    addr &= MTK_IOMMU_SIZE - 1;
    if (addr >= REG_MMU_FAULT_ST_FIRST && addr <= REG_MMU_FAULT_ST_LAST) {
        return 0;   /* no translation faults are ever raised */
    }
    return s->regs[addr >> 2];
}

static void mtk_iommu_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    MtkIommuState *s = opaque;

    addr &= MTK_IOMMU_SIZE - 1;
    switch (addr) {
    case REG_MMU_INVALIDATE:
        /* Self-clearing command register. */
        if (val & (F_MMU_INV_RANGE | F_ALL_INVLD)) {
            s->regs[REG_MMU_CPE_DONE >> 2] = 1;
            s->n_inval++;
            if (s->log && s->n_inval <= 8) {
                fprintf(stderr, "MTK-IOMMU %s inval=0x%x #%u\n",
                        memory_region_name(&s->iomem), (unsigned)val,
                        s->n_inval);
            }
        }
        s->regs[addr >> 2] = 0;
        return;
    case REG_MMU_INT_CONTROL0:
        s->regs[addr >> 2] = val & ~F_INT_CLR_BIT;
        return;
    default:
        if (addr >= REG_MMU_FAULT_ST_FIRST && addr <= REG_MMU_FAULT_ST_LAST) {
            return;
        }
        s->regs[addr >> 2] = val;
        return;
    }
}

static const MemoryRegionOps mtk_iommu_ops = {
    .read = mtk_iommu_read,
    .write = mtk_iommu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void mtk_iommu_reset(DeviceState *dev)
{
    MtkIommuState *s = MTK_IOMMU(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->n_inval = 0;
}

static void mtk_iommu_init(Object *obj)
{
    MtkIommuState *s = MTK_IOMMU(obj);

    memory_region_init_io(&s->iomem, obj, &mtk_iommu_ops, s,
                          "mt6895.iommu", MTK_IOMMU_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    s->log = getenv("MTK_IOMMU_LOG") != NULL;
}

static const VMStateDescription vmstate_mtk_iommu = {
    .name = TYPE_MTK_IOMMU,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, MtkIommuState, MTK_IOMMU_SIZE / 4),
        VMSTATE_UINT32(n_inval, MtkIommuState),
        VMSTATE_END_OF_LIST()
    },
};

static void mtk_iommu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, mtk_iommu_reset);
    dc->vmsd = &vmstate_mtk_iommu;
}

static const TypeInfo mtk_iommu_info = {
    .name = TYPE_MTK_IOMMU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MtkIommuState),
    .instance_init = mtk_iommu_init,
    .class_init = mtk_iommu_class_init,
};

static void mtk_iommu_register_types(void)
{
    type_register_static(&mtk_iommu_info);
}

type_init(mtk_iommu_register_types)
