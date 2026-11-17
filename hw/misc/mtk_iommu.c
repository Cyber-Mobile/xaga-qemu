/*
 * MediaTek M4U / IOMMU for the MT6895 machine model.
 *
 * Semantics are taken from the vendor `mtk_iommu.ko` shipped in the LineageOS
 * vendor_boot ramdisk (vermagic 5.10.269-g56de8342020a) and the matching
 * public kernel source tree (XagaForge/android_kernel_xiaomi_mt6895, branch
 * 17.0, drivers/iommu/mtk_iommu.c + io-pgtable-arm-v7s.c), cross-checked by
 * disassembling the module:
 *
 *   - every M4U block has a 4 KiB register window per bank; Linux only
 *     programs bank 0 (normal/NS domain), banks 1..4 are the protected/secure
 *     banks managed by TF-A through SMC;
 *   - REG_MMU_PT_BASE_ADDR (0x000) receives the TTBR value of the ARMv7
 *     short-descriptor page table, masked with GENMASK(31, 7).  The IOMMU has
 *     no separate enable bit: a non-zero page table base means the bank
 *     translates;
 *   - translation uses the MediaTek extension (IO_PGTABLE_QUIRK_ARM_MTK_EXT):
 *     34-bit IOVA (level 1 index is 14 bits wide, 64 KiB L1 table) and 35-bit
 *     PA, with PA[32..34] carried in page table entry bits 9, 4 and 5;
 *   - REG_MMU_INVALIDATE (0x020) with F_MMU_INV_RANGE/F_ALL_INVLD starts a
 *     TLB invalidation and completion is reported through REG_MMU_CPE_DONE
 *     (0x12c).  The model has no TLB cache (every access walks the page
 *     table), so invalidations complete immediately;
 *   - faults latch into REG_MMU_FAULT_ST0/ST1 (0x130/0x134) and
 *     REG_MMU_FAULT_VA (0x13c) and are reported on the IOMMU interrupt line
 *     when the machine wires it (the MT6895 machine currently does not need
 *     the fault IRQs for recovery display bring-up).
 *
 * The bank-0 instances are registered by (iommu-id, bank) QOM properties so
 * the display/GCE models can translate their DMA addresses through the same
 * page table Linux programs.
 *
 * MTK_IOMMU=1 enables translation (consumers then treat register values as
 * IOVAs); MTK_IOMMU_LOG=1 logs invalidations and translation faults.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/address-spaces.h"
#include "system/memory.h"

#include "hw/misc/mtk_iommu.h"

#define TYPE_MTK_IOMMU "mtk-iommu"
OBJECT_DECLARE_SIMPLE_TYPE(MtkIommuState, MTK_IOMMU)

#define MTK_IOMMU_SIZE            0x1000

/* Bank-0 control/status registers (vendor drivers/iommu/mtk_iommu.c). */
#define REG_MMU_PT_BASE_ADDR      0x000
#define   MMU_PT_ADDR_MASK        0xffffff80u   /* GENMASK(31, 7) */
#define REG_MMU_INVALIDATE        0x020
#define   F_MMU_INV_RANGE         (1u << 0)
#define   F_ALL_INVLD             (1u << 1)
#define REG_MMU_INVLD_START_A     0x024
#define REG_MMU_INVLD_END_A       0x028
#define REG_MMU_INV_SEL_GEN2      0x02c
#define REG_MMU_CTRL_REG          0x110
#define   F_MMU_SYNC_INVLDT_EN    (1u << 3)
#define REG_MMU_IVRP_PADDR        0x114
#define REG_MMU_VLD_PA_RNG        0x118
#define REG_MMU_INT_CONTROL0      0x120
#define   F_INT_CLR_BIT           (1u << 12)
#define REG_MMU_INT_MAIN_CONTROL  0x124
#define REG_MMU_CPE_DONE          0x12c
#define REG_MMU_FAULT_ST0         0x130
#define   F_INT_TBW_FAULT         (1u << 1)
#define REG_MMU_FAULT_ST1         0x134
#define   F_INT_MMU_TF_ERR(MMU)   (1u << (0 + (MMU) * 7))
#define REG_MMU_TBWALK_FAULT_VA   0x138
#define REG_MMU_FAULT_VA          0x13c
#define   F_MMU_FAULT_VA_WRITE_BIT (1u << 1)
#define REG_MMU_INVLD_PA          0x140
#define REG_MMU_INT_ID            0x150

/*
 * ARMv7 short-descriptor page table, MediaTek extension:
 *   - ias = 34: 14 index bits at level 1 (16 K entries, 64 KiB table),
 *               8 index bits at level 2 (256 entries, 1 KiB table);
 *   - 4 KiB small pages (L2, bit1 = 1), 64 KiB contiguous pages (L2, bit1 = 0
 *     and bit0 = 1), 1 MiB sections and 16 MiB supersections (L1, byte1 = 0
 *     and bit0 = 1, bit18 selects supersections);
 *   - PA bits 32/33/34 live in entry bits 9/4/5 (io-pgtable-arm-v7s.c:
 *     ARM_V7S_ATTR_MTK_PA_BIT32/33/34).
 */
#define ARM_V7S_PTE_TYPE_MASK     0x3u
#define ARM_V7S_PTE_TYPE_TABLE    0x1u
#define ARM_V7S_PTE_TYPE_PAGE     0x2u
#define ARM_V7S_LVL1_IDX_SHIFT    20
#define ARM_V7S_LVL1_IDX_MASK     0x3fffu
#define ARM_V7S_LVL2_IDX_SHIFT    12
#define ARM_V7S_LVL2_IDX_MASK     0xffu
#define ARM_V7S_LVL1_PA_MASK      0xfff00000u
#define ARM_V7S_LVL2_PA_MASK      0xfffff000u
#define ARM_V7S_TABLE_PA_MASK     0xfffffc00u
#define ARM_V7S_CONT_SECTION      (1u << 18)
#define ARM_V7S_CONT_PAGES        16
#define ARM_V7S_PTE_BIT_PA32      9
#define ARM_V7S_PTE_BIT_PA33      4
#define ARM_V7S_PTE_BIT_PA34      5

struct MtkIommuState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[MTK_IOMMU_SIZE / 4];
    uint32_t n_inval;
    uint32_t n_fault;
    uint32_t n_xlate_ok;
    uint32_t n_identity;
    bool log;
    /* QOM configuration: which M4U block and which bank this window is. */
    uint32_t iommu_id;
    uint32_t bank;
};

/*
 * Bootloader-physical addresses (LK framebuffer, splash layers) are below
 * 4 GiB and are handed over outside any IOMMU domain; the identity fallback
 * only applies there, never to a >4 GiB IOVA (which must come from the page
 * table).
 */
#define MTK_IOMMU_IDENTITY_LIMIT  0x100000000ULL

/* bank-0 instances, indexed by MTK_IOMMU_DEV_* */
static MtkIommuState *mtk_iommu_bank0[MTK_IOMMU_DEV_NUM];

static int mtk_iommu_configured = -1;

bool mtk_iommu_enabled(void)
{
    if (mtk_iommu_configured < 0) {
        const char *e = getenv("MTK_IOMMU");

        /*
         * MTK_DT_NOIOMMU=1 keeps the old bring-up hack (device tree iommus
         * properties NOPed out, clients program physical addresses), so it
         * wins over MTK_IOMMU=1: the two settings are mutually exclusive.
         */
        mtk_iommu_configured = e != NULL && strcmp(e, "0") != 0 &&
                               getenv("MTK_DT_NOIOMMU") == NULL;
    }
    return mtk_iommu_configured;
}

static bool mtk_iommu_readl(hwaddr addr, uint32_t *val)
{
    MemTxResult result;

    *val = address_space_ldl_le(&address_space_memory, addr,
                                MEMTXATTRS_UNSPECIFIED, &result);
    return result == MEMTX_OK;
}

/* Rebuild the physical address of a page table entry (MTK 35-bit PA). */
static hwaddr mtk_iommu_pte_pa(uint32_t pte, uint32_t mask)
{
    hwaddr pa = pte & mask;

    if (pte & (1u << ARM_V7S_PTE_BIT_PA32)) {
        pa |= 1ULL << 32;
    }
    if (pte & (1u << ARM_V7S_PTE_BIT_PA33)) {
        pa |= 1ULL << 33;
    }
    if (pte & (1u << ARM_V7S_PTE_BIT_PA34)) {
        pa |= 1ULL << 34;
    }
    return pa;
}

static void mtk_iommu_fault(MtkIommuState *s, uint64_t iova, uint32_t l1e,
                            uint32_t l2e, const char *reason)
{
    s->n_fault++;
    /*
     * Latch the fault in the hardware status registers like the M4U does:
     * FAULT_ST0 bit1 (table walk fault), FAULT_ST1 bit0 (translation fault of
     * MMU0), FAULT_VA[31:12] = faulting IOVA, INVLD_PA = faulting entry.
     */
    if (!(s->regs[REG_MMU_FAULT_ST0 >> 2] & F_INT_TBW_FAULT)) {
        s->regs[REG_MMU_FAULT_ST0 >> 2] |= F_INT_TBW_FAULT;
        s->regs[REG_MMU_FAULT_ST1 >> 2] |= F_INT_MMU_TF_ERR(0);
        s->regs[REG_MMU_FAULT_VA >> 2] =
            ((uint32_t)iova & 0xfffff000u) |
            (uint32_t)((iova >> 32) & 0x7) << 9;
        s->regs[REG_MMU_INVLD_PA >> 2] = l2e ? l2e : l1e;
        s->regs[REG_MMU_TBWALK_FAULT_VA >> 2] = l1e;
    }
    if (s->log && s->n_fault <= 24) {
        fprintf(stderr,
                "MTK-IOMMU dev%u: %s iova=%#" PRIx64 " l1e=%#x l2e=%#x "
                "pt=%#x\n", s->iommu_id, reason, iova, l1e, l2e,
                s->regs[REG_MMU_PT_BASE_ADDR >> 2]);
    }
}

/*
 * Walk the bank page table.  Quiet: a miss never latches a fault, because the
 * caller has to decide whether it is a real fault or a bootloader-physical
 * buffer that the identity fallback covers (LK programs the OVL/GCE windows
 * with physical addresses before Linux attaches an IOMMU domain, and those
 * addresses stay below 4 GiB while the active IOVA space starts at 0x1000).
 * The entries seen on the way are returned for fault reporting.
 */
static bool mtk_iommu_walk(MtkIommuState *s, uint64_t iova, hwaddr *pa,
                           uint32_t *l1e_out, uint32_t *l2e_out)
{
    uint32_t pt, l1e = 0, l2e = 0, idx;

    *l1e_out = 0;
    *l2e_out = 0;
    pt = s->regs[REG_MMU_PT_BASE_ADDR >> 2] & MMU_PT_ADDR_MASK;
    if (!pt) {
        return false;
    }

    idx = (iova >> ARM_V7S_LVL1_IDX_SHIFT) & ARM_V7S_LVL1_IDX_MASK;
    if (!mtk_iommu_readl(pt + idx * 4, &l1e)) {
        return false;
    }
    *l1e_out = l1e;
    if ((l1e & ARM_V7S_PTE_TYPE_MASK) == 0) {
        return false;
    }

    if ((l1e & ARM_V7S_PTE_TYPE_MASK) == ARM_V7S_PTE_TYPE_TABLE) {
        hwaddr l2pt = mtk_iommu_pte_pa(l1e, ARM_V7S_TABLE_PA_MASK);

        idx = (iova >> ARM_V7S_LVL2_IDX_SHIFT) & ARM_V7S_LVL2_IDX_MASK;
        if (!mtk_iommu_readl(l2pt + idx * 4, &l2e)) {
            return false;
        }
        *l2e_out = l2e;
        if ((l2e & ARM_V7S_PTE_TYPE_MASK) == 0) {
            return false;
        }
        if (l2e & ARM_V7S_PTE_TYPE_PAGE) {
            /* 4 KiB small page */
            *pa = mtk_iommu_pte_pa(l2e, ARM_V7S_LVL2_PA_MASK) |
                  ((uint32_t)iova & 0xfff);
        } else {
            /* 64 KiB contiguous page: 16 repeated small page entries */
            uint32_t mask = (uint32_t)(ARM_V7S_LVL2_PA_MASK *
                                       ARM_V7S_CONT_PAGES);
            *pa = mtk_iommu_pte_pa(l2e, mask) | (iova & ~(hwaddr)mask);
        }
        return true;
    }

    /* Section (1 MiB) or supersection (16 MiB). */
    if (l1e & ARM_V7S_CONT_SECTION) {
        uint32_t mask = (uint32_t)(ARM_V7S_LVL1_PA_MASK * ARM_V7S_CONT_PAGES);

        *pa = mtk_iommu_pte_pa(l1e, mask) | (iova & ~(hwaddr)mask);
    } else {
        *pa = mtk_iommu_pte_pa(l1e, ARM_V7S_LVL1_PA_MASK) |
              ((uint32_t)iova & 0xfffff);
    }
    return true;
}

/* Physical buffer that never went through an IOMMU domain (bootloader path). */
static bool mtk_iommu_identity(uint64_t dma, hwaddr *pa)
{
    if (dma >= MTK_IOMMU_IDENTITY_LIMIT ||
        !memory_region_present(get_system_memory(), dma)) {
        return false;
    }
    *pa = dma;
    return true;
}

bool mtk_iommu_translate(uint16_t dev_id, uint64_t iova, hwaddr *pa)
{
    MtkIommuState *s;

    if (!mtk_iommu_enabled() || dev_id >= MTK_IOMMU_DEV_NUM) {
        return false;
    }
    s = mtk_iommu_bank0[dev_id];
    if (!s) {
        return false;
    }
    uint32_t l1e = 0, l2e = 0;

    /* A page table hit always wins: kernel-allocated IOVAs live there. */
    if (s->regs[REG_MMU_PT_BASE_ADDR >> 2] &&
        mtk_iommu_walk(s, iova, pa, &l1e, &l2e)) {
        s->n_xlate_ok++;
        if (s->log && s->n_xlate_ok <= 4) {
            fprintf(stderr,
                    "MTK-IOMMU dev%u: iova=%#" PRIx64 " -> pa=%#" HWADDR_PRIx
                    "\n", s->iommu_id, iova, *pa);
        }
        return true;
    }
    /*
     * Page table absent or entry empty: bootloader-physical buffers (LK
     * framebuffer/splash, < 4 GiB and backed by guest memory) are handed over
     * without any IOMMU mapping and must read through as-is.  Only addresses
     * that are outside that window (or not backed at all) are real faults.
     */
    if (mtk_iommu_identity(iova, pa)) {
        s->n_identity++;
        if (s->log && s->n_identity <= 8) {
            fprintf(stderr, "MTK-IOMMU dev%u: identity %#" PRIx64
                    " (bootloader-physical buffer)\n", s->iommu_id, iova);
        }
        return true;
    }
    mtk_iommu_fault(s, iova, l1e, l2e, "unmapped IOVA");
    return false;
}

bool mtk_iommu_translate_or_identity(uint16_t dev_id, uint64_t dma,
                                     hwaddr *pa)
{
    if (!mtk_iommu_enabled()) {
        *pa = dma;
        return true;
    }
    return mtk_iommu_translate(dev_id, dma, pa);
}

static uint64_t mtk_iommu_read(void *opaque, hwaddr addr, unsigned size)
{
    MtkIommuState *s = opaque;

    addr &= MTK_IOMMU_SIZE - 1;
    return s->regs[addr >> 2];
}

static void mtk_iommu_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    MtkIommuState *s = opaque;

    addr &= MTK_IOMMU_SIZE - 1;
    switch (addr) {
    case REG_MMU_PT_BASE_ADDR:
        s->regs[addr >> 2] = val;
        if (s->log) {
            fprintf(stderr, "MTK-IOMMU dev%u: pt_base=0x%x\n",
                    s->iommu_id, (unsigned)(val & 0xffffffffu));
        }
        return;
    case REG_MMU_INVALIDATE:
        /* Self-clearing command register. */
        if (val & (F_MMU_INV_RANGE | F_ALL_INVLD)) {
            s->regs[REG_MMU_CPE_DONE >> 2] = 1;
            s->n_inval++;
            if (s->log && s->n_inval <= 8) {
                fprintf(stderr, "MTK-IOMMU dev%u inval=0x%x #%u\n",
                        s->iommu_id, (unsigned)val, s->n_inval);
            }
        }
        s->regs[addr >> 2] = 0;
        return;
    case REG_MMU_INT_CONTROL0:
        s->regs[addr >> 2] = val & ~F_INT_CLR_BIT;
        return;
    default:
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
    s->n_fault = 0;
}

static void mtk_iommu_realize(DeviceState *dev, Error **errp)
{
    MtkIommuState *s = MTK_IOMMU(dev);

    s->log = getenv("MTK_IOMMU_LOG") != NULL;
    if (s->iommu_id < MTK_IOMMU_DEV_NUM && s->bank == 0) {
        mtk_iommu_bank0[s->iommu_id] = s;
        if (mtk_iommu_enabled()) {
            fprintf(stderr, "MTK-IOMMU: dev%u bank0 translation enabled "
                    "(MTK_IOMMU=1)\n", s->iommu_id);
        }
    }
}

static void mtk_iommu_init(Object *obj)
{
    MtkIommuState *s = MTK_IOMMU(obj);

    memory_region_init_io(&s->iomem, obj, &mtk_iommu_ops, s,
                          "mt6895.iommu", MTK_IOMMU_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const Property mtk_iommu_properties[] = {
    DEFINE_PROP_UINT32("iommu-id", MtkIommuState, iommu_id, MTK_IOMMU_DEV_NUM),
    DEFINE_PROP_UINT32("bank", MtkIommuState, bank, 0),
};

static const VMStateDescription vmstate_mtk_iommu = {
    .name = TYPE_MTK_IOMMU,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, MtkIommuState, MTK_IOMMU_SIZE / 4),
        VMSTATE_UINT32(n_inval, MtkIommuState),
        VMSTATE_UINT32(n_fault, MtkIommuState),
        VMSTATE_END_OF_LIST()
    },
};

static void mtk_iommu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, mtk_iommu_reset);
    device_class_set_props(dc, mtk_iommu_properties);
    dc->realize = mtk_iommu_realize;
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
