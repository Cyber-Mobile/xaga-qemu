/*
 * MediaTek MT6895 Arm Mali-G610 MC6 control register stub
 *
 * Exposes the Mali-G610 hardware identification, feature masks, GPU/MMU/JOB
 * interrupt registers, and core power-ready states for early driver bring-up.
 * CSF firmware startup and queue execution remain pending.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "hw/misc/mtk_mali_gpu.h"
#include "system/memory.h"

#define MALI_REG_WINDOW         0x4000u
#define MALI_REG_WORDS          (MALI_REG_WINDOW / 4u)

#define MALI_GPU_ID             0x000u
#define MALI_L2_FEATURES        0x004u
#define MALI_CORE_FEATURES      0x008u
#define MALI_TILER_FEATURES     0x00cu
#define MALI_MEM_FEATURES       0x010u
#define MALI_MMU_FEATURES       0x014u
#define MALI_AS_PRESENT         0x018u
#define MALI_JS_PRESENT         0x01cu

#define MALI_GPU_IRQ_RAWSTAT    0x020u
#define MALI_GPU_IRQ_CLEAR      0x024u
#define MALI_GPU_IRQ_MASK       0x028u
#define MALI_GPU_IRQ_STATUS     0x02cu
#define MALI_GPU_COMMAND        0x030u
#define MALI_GPU_STATUS         0x034u

#define MALI_THREAD_MAX_THREADS 0x0a0u
#define MALI_THREAD_MAX_WG      0x0a4u
#define MALI_THREAD_MAX_BARRIER 0x0a8u
#define MALI_THREAD_FEATURES    0x0acu
#define MALI_TEXTURE_FEAT0      0x0b0u
#define MALI_TEXTURE_FEAT3      0x0bcu
#define MALI_THREAD_TLS_ALLOC   0x0e0u

#define MALI_SHADER_PRESENT_LO  0x100u
#define MALI_SHADER_PRESENT_HI  0x104u
#define MALI_TILER_PRESENT_LO   0x110u
#define MALI_TILER_PRESENT_HI   0x114u
#define MALI_L2_PRESENT_LO      0x120u
#define MALI_L2_PRESENT_HI      0x124u

#define MALI_SHADER_READY_LO    0x140u
#define MALI_TILER_READY_LO     0x150u
#define MALI_L2_READY_LO        0x160u
#define MALI_SHADER_PWRON_LO    0x180u
#define MALI_TILER_PWRON_LO     0x190u
#define MALI_L2_PWRON_LO        0x1a0u
#define MALI_SHADER_PWROFF_LO   0x1c0u
#define MALI_TILER_PWROFF_LO    0x1d0u
#define MALI_L2_PWROFF_LO       0x1e0u

#define MALI_COHERENCY_FEATURES 0x300u
#define MALI_STACK_PRESENT_LO   0x310u

#define MALI_JOB_IRQ_RAWSTAT    0x1000u
#define MALI_JOB_IRQ_CLEAR      0x1004u
#define MALI_JOB_IRQ_MASK       0x1008u
#define MALI_JOB_IRQ_STATUS     0x100cu

#define MALI_MMU_IRQ_RAWSTAT    0x2000u
#define MALI_MMU_IRQ_CLEAR      0x2004u
#define MALI_MMU_IRQ_MASK       0x2008u
#define MALI_MMU_IRQ_STATUS     0x200cu

#define MALI_G610_GPU_ID        0xa8670000u
#define MALI_SHADER_MASK        0x0000003fu
#define MALI_GPU_IRQ_RESET_DONE BIT(8)
#define MALI_GPU_IRQ_PWR_CHG    (3u << 9)
#define MALI_GPU_IRQ_CLEAN_DONE BIT(17)

struct MtkMaliGpuState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq_gpu;
    qemu_irq irq_mmu;
    qemu_irq irq_job;

    uint32_t gpu_irq_rawstat;
    uint32_t gpu_irq_mask;
    uint32_t mmu_irq_rawstat;
    uint32_t mmu_irq_mask;
    uint32_t job_irq_rawstat;
    uint32_t job_irq_mask;

    uint32_t shader_ready;
    uint32_t tiler_ready;
    uint32_t l2_ready;

    uint32_t regs[MALI_REG_WORDS];
};

static void mali_update_irqs(MtkMaliGpuState *s)
{
    qemu_set_irq(s->irq_gpu, (s->gpu_irq_rawstat & s->gpu_irq_mask) != 0);
    qemu_set_irq(s->irq_mmu, (s->mmu_irq_rawstat & s->mmu_irq_mask) != 0);
    qemu_set_irq(s->irq_job, (s->job_irq_rawstat & s->job_irq_mask) != 0);
}

static uint32_t mali_read_word(MtkMaliGpuState *s, hwaddr off)
{
    if (off >= MALI_REG_WINDOW) {
        return 0;
    }

    switch (off) {
    case MALI_GPU_ID:
        return MALI_G610_GPU_ID;
    case MALI_L2_FEATURES:
        return 0x07130206u;
    case MALI_CORE_FEATURES:
        return 0x00000000u;
    case MALI_TILER_FEATURES:
        return 0x00000809u;
    case MALI_MEM_FEATURES:
        return 0x00000001u;
    case MALI_MMU_FEATURES:
        return 0x00002830u;
    case MALI_AS_PRESENT:
        return 0x000000ffu;
    case MALI_JS_PRESENT:
        return 0x00000000u;
    case MALI_THREAD_MAX_THREADS:
    case MALI_THREAD_MAX_WG:
    case MALI_THREAD_MAX_BARRIER:
    case MALI_THREAD_TLS_ALLOC:
        return 0x00000100u;
    case MALI_THREAD_FEATURES:
        return 0x0a040000u;
    case MALI_TEXTURE_FEAT0 ... MALI_TEXTURE_FEAT3:
        return 0x0001feffu;
    case MALI_SHADER_PRESENT_LO:
        return MALI_SHADER_MASK;
    case MALI_SHADER_PRESENT_HI:
        return 0x00000000u;
    case MALI_TILER_PRESENT_LO:
        return 0x00000001u;
    case MALI_TILER_PRESENT_HI:
        return 0x00000000u;
    case MALI_L2_PRESENT_LO:
        return 0x00000001u;
    case MALI_L2_PRESENT_HI:
        return 0x00000000u;
    case MALI_COHERENCY_FEATURES:
        return 0x00000001u;
    case MALI_STACK_PRESENT_LO:
        return 0x00000003u;
    case MALI_GPU_IRQ_RAWSTAT:
        return s->gpu_irq_rawstat;
    case MALI_GPU_IRQ_MASK:
        return s->gpu_irq_mask;
    case MALI_GPU_IRQ_STATUS:
        return s->gpu_irq_rawstat & s->gpu_irq_mask;
    case MALI_GPU_STATUS:
        return 0x00000000u;
    case MALI_SHADER_READY_LO:
        return s->shader_ready;
    case MALI_TILER_READY_LO:
        return s->tiler_ready;
    case MALI_L2_READY_LO:
        return s->l2_ready;
    case MALI_JOB_IRQ_RAWSTAT:
        return s->job_irq_rawstat;
    case MALI_JOB_IRQ_MASK:
        return s->job_irq_mask;
    case MALI_JOB_IRQ_STATUS:
        return s->job_irq_rawstat & s->job_irq_mask;
    case MALI_MMU_IRQ_RAWSTAT:
        return s->mmu_irq_rawstat;
    case MALI_MMU_IRQ_MASK:
        return s->mmu_irq_mask;
    case MALI_MMU_IRQ_STATUS:
        return s->mmu_irq_rawstat & s->mmu_irq_mask;
    default:
        return s->regs[off >> 2];
    }
}

static void mali_write_word(MtkMaliGpuState *s, hwaddr off, uint32_t val)
{
    uint32_t cmd;

    if (off >= MALI_REG_WINDOW) {
        return;
    }

    switch (off) {
    case MALI_GPU_IRQ_RAWSTAT:
        s->gpu_irq_rawstat |= val;
        mali_update_irqs(s);
        break;
    case MALI_GPU_IRQ_CLEAR:
        s->gpu_irq_rawstat &= ~val;
        mali_update_irqs(s);
        break;
    case MALI_GPU_IRQ_MASK:
        s->gpu_irq_mask = val;
        mali_update_irqs(s);
        break;
    case MALI_GPU_COMMAND:
        cmd = val & 0xffu;
        if (cmd == 0x01u || cmd == 0x02u) {
            s->shader_ready = 0;
            s->tiler_ready = 0;
            s->l2_ready = 0;
            s->gpu_irq_rawstat |= MALI_GPU_IRQ_RESET_DONE;
            mali_update_irqs(s);
        } else if (cmd == 0x03u) {
            s->gpu_irq_rawstat |= MALI_GPU_IRQ_CLEAN_DONE;
            mali_update_irqs(s);
        }
        break;
    case MALI_SHADER_PWRON_LO:
        s->shader_ready |= (val & MALI_SHADER_MASK);
        s->gpu_irq_rawstat |= MALI_GPU_IRQ_PWR_CHG;
        mali_update_irqs(s);
        break;
    case MALI_TILER_PWRON_LO:
        s->tiler_ready |= (val & 0x1u);
        s->gpu_irq_rawstat |= MALI_GPU_IRQ_PWR_CHG;
        mali_update_irqs(s);
        break;
    case MALI_L2_PWRON_LO:
        s->l2_ready |= (val & 0x1u);
        s->gpu_irq_rawstat |= MALI_GPU_IRQ_PWR_CHG;
        mali_update_irqs(s);
        break;
    case MALI_SHADER_PWROFF_LO:
        s->shader_ready &= ~val;
        s->gpu_irq_rawstat |= MALI_GPU_IRQ_PWR_CHG;
        mali_update_irqs(s);
        break;
    case MALI_TILER_PWROFF_LO:
        s->tiler_ready &= ~val;
        s->gpu_irq_rawstat |= MALI_GPU_IRQ_PWR_CHG;
        mali_update_irqs(s);
        break;
    case MALI_L2_PWROFF_LO:
        s->l2_ready &= ~val;
        s->gpu_irq_rawstat |= MALI_GPU_IRQ_PWR_CHG;
        mali_update_irqs(s);
        break;
    case MALI_JOB_IRQ_RAWSTAT:
        s->job_irq_rawstat |= val;
        mali_update_irqs(s);
        break;
    case MALI_JOB_IRQ_CLEAR:
        s->job_irq_rawstat &= ~val;
        mali_update_irqs(s);
        break;
    case MALI_JOB_IRQ_MASK:
        s->job_irq_mask = val;
        mali_update_irqs(s);
        break;
    case MALI_MMU_IRQ_RAWSTAT:
        s->mmu_irq_rawstat |= val;
        mali_update_irqs(s);
        break;
    case MALI_MMU_IRQ_CLEAR:
        s->mmu_irq_rawstat &= ~val;
        mali_update_irqs(s);
        break;
    case MALI_MMU_IRQ_MASK:
        s->mmu_irq_mask = val;
        mali_update_irqs(s);
        break;
    default:
        s->regs[off >> 2] = val;
        break;
    }
}

static uint64_t mtk_mali_gpu_read(void *opaque, hwaddr offset, unsigned size)
{
    MtkMaliGpuState *s = MTK_MALI_GPU(opaque);
    uint64_t out = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        hwaddr o = offset + i;
        if (o < MALI_REG_WINDOW) {
            uint32_t w = mali_read_word(s, o & ~3ULL);
            out |= (uint64_t)((w >> (8 * (o & 3))) & 0xffu) << (8 * i);
        }
    }
    return out;
}

static void mtk_mali_gpu_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    MtkMaliGpuState *s = MTK_MALI_GPU(opaque);
    hwaddr word_off = offset & ~3ULL;
    uint32_t cur, mask, shift, new_val;

    if (word_off >= MALI_REG_WINDOW) {
        return;
    }
    if (size == 4 && (offset & 3) == 0) {
        mali_write_word(s, word_off, (uint32_t)value);
        return;
    }

    shift = (unsigned)(offset & 3) * 8;
    mask = (size >= 4) ? 0xffffffffu : ((1u << (size * 8)) - 1u);
    cur = mali_read_word(s, word_off);
    new_val = (cur & ~(mask << shift)) | (((uint32_t)value & mask) << shift);
    mali_write_word(s, word_off, new_val);
}

static const MemoryRegionOps mtk_mali_gpu_ops = {
    .read = mtk_mali_gpu_read,
    .write = mtk_mali_gpu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static void mtk_mali_gpu_reset(DeviceState *dev)
{
    MtkMaliGpuState *s = MTK_MALI_GPU(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->gpu_irq_rawstat = 0;
    s->gpu_irq_mask = 0;
    s->mmu_irq_rawstat = 0;
    s->mmu_irq_mask = 0;
    s->job_irq_rawstat = 0;
    s->job_irq_mask = 0;
    s->shader_ready = 0;
    s->tiler_ready = 0;
    s->l2_ready = 0;
    mali_update_irqs(s);
}

static void mtk_mali_gpu_realize(DeviceState *dev, Error **errp)
{
    MtkMaliGpuState *s = MTK_MALI_GPU(dev);

    (void)errp;
    memory_region_init_io(&s->iomem, OBJECT(dev), &mtk_mali_gpu_ops, s,
                          TYPE_MTK_MALI_GPU, MTK_MALI_GPU_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq_gpu);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq_mmu);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq_job);
}

static void mtk_mali_gpu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    (void)data;
    dc->realize = mtk_mali_gpu_realize;
    dc->desc = "MediaTek MT6895 Arm Mali-G610 control registers";
    device_class_set_legacy_reset(dc, mtk_mali_gpu_reset);
}

static const TypeInfo mtk_mali_gpu_info = {
    .name = TYPE_MTK_MALI_GPU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MtkMaliGpuState),
    .class_init = mtk_mali_gpu_class_init,
};

static void mtk_mali_gpu_register_types(void)
{
    type_register_static(&mtk_mali_gpu_info);
}

type_init(mtk_mali_gpu_register_types)
