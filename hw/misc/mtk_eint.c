/*
 * MediaTek MT6895 GPIO/EINT (external interrupt) controller model.
 *
 * The vendor pinctrl driver binds to the "mediatek,mt6895-pinctrl" node and to
 * a separate "mediatek,mt6983-eint" node (apirq@11e50000).  The EINT block is
 * what turns pin level changes into GIC interrupts:
 *
 *   - five instances (w/e/s/n/c) at 0x11e50000, 0x11ce0000, 0x11de0000,
 *     0x11fe0000 and 0x1c01e000, one 0x1000 register bank each;
 *   - all five share one GIC SPI (259, from the device tree);
 *   - register layout from drivers/pinctrl/mediatek/mtk-eint.c
 *     (mtk_generic_eint_regs): stat 0x000, ack 0x040, mask 0x080,
 *     mask_set 0x0c0, mask_clr 0x100, sens 0x140, sens_set 0x180,
 *     sens_clr 0x1c0, soft 0x200, ..., dom_en 0x400, dbnc_*, raw_stat 0xa00.
 *     Each register covers 32 EINTs and the word is selected by
 *     (eint_index / 32) * 4.
 *
 * The model keeps a latched pending bitmap per instance.  Writes to the ACK
 * register clear pending bits; for instance 4 (eint_c) the vendor workaround
 * in mt6983_eint_ack() acknowledges through sens_clr then sens_set instead, so
 * those writes clear the latch as well.  The GIC line stays asserted while any
 * unmasked pending bit is set, matching the level-triggered wiring in the DT.
 *
 * Pin 243 is the SPMI PMIC RCS interrupt (DT: <243 4 16 0> = instance 4,
 * index 16).  hw/misc/mtk_spmi_pmif.c drives that pin through the device's
 * GPIO input line 0, i.e. the PMIC power key interrupt reaches the guest the
 * same way it does on hardware.
 *
 * Env: MTK_EINTLOG=1 traces register accesses and pending changes on stderr.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qom/object.h"

#define TYPE_MTK_EINT "mtk-eint"
OBJECT_DECLARE_SIMPLE_TYPE(MtkEintState, MTK_EINT)

#define MTK_EINT_INSTANCES      5
#define MTK_EINT_SIZE           0x1000
#define MTK_EINT_NREGS          (MTK_EINT_SIZE / 4)

#define MTK_EINT_STAT           0x000
#define MTK_EINT_ACK            0x040
#define MTK_EINT_MASK           0x080
#define MTK_EINT_MASK_SET       0x0c0
#define MTK_EINT_MASK_CLR       0x100
#define MTK_EINT_SENS_SET       0x180
#define MTK_EINT_SENS_CLR       0x1c0
#define MTK_EINT_RAW_STAT       0xa00

typedef struct MtkEintMr {
    MemoryRegion mr;
    struct MtkEintState *s;
    unsigned inst;
} MtkEintMr;

struct MtkEintState {
    SysBusDevice parent_obj;

    MtkEintMr bank[MTK_EINT_INSTANCES];
    qemu_irq irq;

    /* Plain register file for the registers without modelled behaviour. */
    uint32_t regs[MTK_EINT_INSTANCES][MTK_EINT_NREGS];
    /* Latched interrupt state, active high. */
    uint32_t pend[MTK_EINT_INSTANCES][MTK_EINT_NREGS];
    /* Per-port interrupt mask, 1 = masked. */
    uint32_t mask[MTK_EINT_INSTANCES][MTK_EINT_NREGS];

    /* Pin wired to GPIO input line 0 (SPMI PMIC RCS line). */
    uint32_t rcs_instance;
    uint32_t rcs_index;

    bool trace;
};

static void mtk_eint_update_irq(MtkEintState *s)
{
    bool level = false;
    unsigned i, p;

    for (i = 0; i < MTK_EINT_INSTANCES && !level; i++) {
        for (p = 0; p < MTK_EINT_NREGS; p++) {
            if (s->pend[i][p] & ~s->mask[i][p]) {
                level = true;
                break;
            }
        }
    }

    qemu_set_irq(s->irq, level);
}

static uint64_t mtk_eint_read(void *opaque, hwaddr off, unsigned size)
{
    MtkEintMr *m = opaque;
    MtkEintState *s = m->s;
    unsigned inst = m->inst;
    uint32_t val;

    if (off + 4 > MTK_EINT_SIZE) {
        return 0;
    }

    if (off >= MTK_EINT_RAW_STAT) {
        val = s->pend[inst][(off - MTK_EINT_RAW_STAT) / 4];
    } else if (off < MTK_EINT_MASK) {
        val = s->pend[inst][off / 4];
    } else if (off < MTK_EINT_MASK_SET) {
        val = s->mask[inst][(off - MTK_EINT_MASK) / 4];
    } else {
        val = s->regs[inst][off / 4];
    }

    if (s->trace) {
        fprintf(stderr, "MTK-EINT inst%u read  +0x%03x -> 0x%08x\n",
                inst, (unsigned)off, val);
    }
    return val;
}

static void mtk_eint_write(void *opaque, hwaddr off, uint64_t value,
                           unsigned size)
{
    MtkEintMr *m = opaque;
    MtkEintState *s = m->s;
    unsigned inst = m->inst;
    uint32_t v = value;

    if (off + 4 > MTK_EINT_SIZE) {
        return;
    }

    if (off >= MTK_EINT_ACK && off < MTK_EINT_MASK) {
        s->pend[inst][(off - MTK_EINT_ACK) / 4] &= ~v;
    } else if (off >= MTK_EINT_MASK_SET && off < MTK_EINT_MASK_CLR) {
        s->mask[inst][(off - MTK_EINT_MASK_SET) / 4] |= v;
    } else if (off >= MTK_EINT_MASK_CLR && off < 0x140) {
        s->mask[inst][(off - MTK_EINT_MASK_CLR) / 4] &= ~v;
    } else if (off >= MTK_EINT_SENS_SET && off < MTK_EINT_SENS_CLR) {
        s->regs[inst][off / 4] = v;
        s->pend[inst][(off - MTK_EINT_SENS_SET) / 4] &= ~v;
    } else if (off >= MTK_EINT_SENS_CLR && off < 0x200) {
        s->regs[inst][off / 4] = v;
        s->pend[inst][(off - MTK_EINT_SENS_CLR) / 4] &= ~v;
    } else if (off < MTK_EINT_ACK) {
        /* STAT is read-only. */
    } else {
        s->regs[inst][off / 4] = v;
    }

    if (s->trace) {
        fprintf(stderr, "MTK-EINT inst%u write +0x%03x <- 0x%08x\n",
                inst, (unsigned)off, v);
    }
    mtk_eint_update_irq(s);
}

static const MemoryRegionOps mtk_eint_ops = {
    .read = mtk_eint_read,
    .write = mtk_eint_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/* GPIO input 0: the SPMI PMIC RCS interrupt line (pin 243). */
static void mtk_eint_line_in(void *opaque, int n, int level)
{
    MtkEintState *s = MTK_EINT(opaque);
    unsigned inst = s->rcs_instance;
    unsigned index = s->rcs_index;
    uint32_t bit;

    if (inst >= MTK_EINT_INSTANCES || index >= 32 * MTK_EINT_NREGS) {
        return;
    }
    bit = 1u << (index & 31);
    if (level) {
        s->pend[inst][index >> 5] |= bit;
    } else {
        s->pend[inst][index >> 5] &= ~bit;
    }
    if (s->trace) {
        fprintf(stderr, "MTK-EINT line %d -> %d (inst%u bit%u)\n",
                n, level, inst, index & 31);
    }
    mtk_eint_update_irq(s);
}

static void mtk_eint_reset(DeviceState *dev)
{
    MtkEintState *s = MTK_EINT(dev);
    unsigned i, p;

    for (i = 0; i < MTK_EINT_INSTANCES; i++) {
        for (p = 0; p < MTK_EINT_NREGS; p++) {
            s->regs[i][p] = 0;
            s->pend[i][p] = 0;
            s->mask[i][p] = 0;
        }
    }
    mtk_eint_update_irq(s);
}

static void mtk_eint_init(Object *obj)
{
    MtkEintState *s = MTK_EINT(obj);
    unsigned i;

    for (i = 0; i < MTK_EINT_INSTANCES; i++) {
        s->bank[i].s = s;
        s->bank[i].inst = i;
    }
    s->trace = getenv("MTK_EINTLOG") != NULL;
}

static void mtk_eint_realize(DeviceState *dev, Error **errp)
{
    MtkEintState *s = MTK_EINT(dev);
    unsigned i;

    for (i = 0; i < MTK_EINT_INSTANCES; i++) {
        memory_region_init_io(&s->bank[i].mr, OBJECT(s), &mtk_eint_ops,
                              &s->bank[i], TYPE_MTK_EINT ".bank", MTK_EINT_SIZE);
        sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->bank[i].mr);
    }
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    qdev_init_gpio_in(dev, mtk_eint_line_in, 1);
}

static void mtk_eint_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    static const Property props[] = {
        DEFINE_PROP_UINT32("rcs-instance", MtkEintState, rcs_instance, 4),
        DEFINE_PROP_UINT32("rcs-index", MtkEintState, rcs_index, 16),
    };

    dc->realize = mtk_eint_realize;
    device_class_set_legacy_reset(dc, mtk_eint_reset);
    device_class_set_props(dc, props);
    dc->desc = "MediaTek MT6895 EINT controller";
}

static const TypeInfo mtk_eint_info = {
    .name = TYPE_MTK_EINT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MtkEintState),
    .instance_init = mtk_eint_init,
    .class_init = mtk_eint_class_init,
};

static void mtk_eint_register_types(void)
{
    type_register_static(&mtk_eint_info);
}

type_init(mtk_eint_register_types)
