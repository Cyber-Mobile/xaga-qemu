/*
 * MediaTek SoC system timer (SYST), e.g. MT6765/MT6895 "mediatek,mt6765-timer".
 *
 * Register layout (13 MHz domain):
 *   0x00 CNTCR      counter control  (reads 1: counter enabled)
 *   0x04 CNTSR      counter status   (reads 1)
 *   0x08 CNTCV_L    free-running 64-bit counter, low word
 *   0x0c CNTCV_H    free-running 64-bit counter, high word
 *   0x40 SYST_CON   bit0 EN, bit1 IRQ_EN, bit4 IRQ_CLR (write-1-to-clear)
 *   0x44 SYST_VAL   timeout in ticks; writing it (with EN set) starts the
 *                   countdown.
 *
 * Semantics follow Linux drivers/clocksource/timer-mediatek.c (SYST part):
 *   next_event: CON=EN; VAL=ticks; CON=EN|IRQ_EN
 *   ack:        CON=EN; CON=EN|IRQ_CLR
 *   shutdown:   ack; CON=0
 * When the countdown reaches zero the interrupt status latches; the level
 * interrupt output is asserted while status && IRQ_EN.  IRQ_CLR clears the
 * status.  The counter registers keep the previous QEMU behaviour (derived
 * from QEMU_CLOCK_VIRTUAL, writes ignored).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/host-utils.h"
#include "qemu/error-report.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "hw/timer/mtk_systimer.h"

#define SYST_CNTCR      0x00
#define SYST_CNTSR      0x04
#define SYST_CNTCV_L    0x08
#define SYST_CNTCV_H    0x0c
#define SYST_CON        0x40
#define SYST_VAL        0x44

#define SYST_CON_EN       BIT(0)
#define SYST_CON_IRQ_EN   BIT(1)
#define SYST_CON_IRQ_CLR  BIT(4)

struct MtkSysTimerState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    QEMUTimer *timer;
    qemu_irq irq;

    uint32_t size;
    uint32_t freq_hz;

    uint32_t con;          /* EN | IRQ_EN as last written */
    uint32_t val;          /* last written timeout */
    int64_t deadline_ns;   /* -1: countdown not running */
    bool status;           /* interrupt status latched at expiry */
    uint32_t log_left;     /* MTK_SYSTLOG: remaining accesses to log */
};

static uint64_t mtk_systimer_ticks(MtkSysTimerState *s, int64_t ns)
{
    return muldiv64(ns, s->freq_hz, NANOSECONDS_PER_SECOND);
}

static int64_t mtk_systimer_ticks_to_ns(MtkSysTimerState *s, uint64_t t)
{
    return muldiv64(t, NANOSECONDS_PER_SECOND, s->freq_hz);
}

static void mtk_systimer_update_irq(MtkSysTimerState *s)
{
    qemu_set_irq(s->irq, s->status && (s->con & SYST_CON_IRQ_EN));
}

static void mtk_systimer_stop(MtkSysTimerState *s)
{
    s->deadline_ns = -1;
    timer_del(s->timer);
}

static void mtk_systimer_expire(void *opaque)
{
    MtkSysTimerState *s = opaque;

    s->deadline_ns = -1;
    if (s->con & SYST_CON_EN) {
        s->status = true;
    }
    mtk_systimer_update_irq(s);
}

static void mtk_systimer_start(MtkSysTimerState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    s->deadline_ns = now + mtk_systimer_ticks_to_ns(s, s->val);
    timer_mod(s->timer, s->deadline_ns);
}

static void mtk_systimer_log(MtkSysTimerState *s, const char *op,
                             hwaddr off, uint64_t v)
{
    if (s->log_left) {
        s->log_left--;
        info_report("mtk-systimer: %s +0x%02" HWADDR_PRIx " = 0x%" PRIx64
                    " (con=%#x status=%d t=%" PRId64 "ns)", op, off, v,
                    s->con, s->status,
                    qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
}

static uint64_t mtk_systimer_read(void *opaque, hwaddr off, unsigned size)
{
    MtkSysTimerState *s = opaque;
    uint64_t ticks;
    uint64_t r;

    switch (off) {
    case SYST_CNTCR:
    case SYST_CNTSR:
        return 1;
    case SYST_CNTCV_L:
        ticks = mtk_systimer_ticks(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        return size == 8 ? ticks : (uint32_t)ticks;
    case SYST_CNTCV_H:
        ticks = mtk_systimer_ticks(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        return (uint32_t)(ticks >> 32);
    case SYST_CON:
        r = s->con | (s->status ? SYST_CON_IRQ_CLR : 0);
        mtk_systimer_log(s, "rd", off, r);
        return r;
    case SYST_VAL:
        if (s->deadline_ns >= 0) {
            int64_t left = s->deadline_ns -
                           qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            r = left > 0 ? mtk_systimer_ticks(s, left) : 0;
        } else {
            r = 0;
        }
        mtk_systimer_log(s, "rd", off, r);
        return r;
    default:
        return 0;
    }
}

static void mtk_systimer_write_con(MtkSysTimerState *s, uint32_t v)
{
    if (v & SYST_CON_IRQ_CLR) {
        s->status = false;
    }
    s->con = v & (SYST_CON_EN | SYST_CON_IRQ_EN);
    if (!(s->con & SYST_CON_EN)) {
        /* Clock disabled: countdown stops. */
        mtk_systimer_stop(s);
    }
    mtk_systimer_update_irq(s);
}

static void mtk_systimer_write_val(MtkSysTimerState *s, uint32_t v)
{
    if (!(s->con & SYST_CON_EN)) {
        /* Timeout ticks can only be updated while the clock is enabled. */
        return;
    }
    s->val = v;
    /* Countdown (re)starts once the timeout is updated. */
    mtk_systimer_start(s);
}

static void mtk_systimer_write(void *opaque, hwaddr off, uint64_t v,
                               unsigned size)
{
    MtkSysTimerState *s = opaque;

    mtk_systimer_log(s, "wr", off, v);
    switch (off) {
    case SYST_CON:
        mtk_systimer_write_con(s, (uint32_t)v);
        if (size == 8) {
            mtk_systimer_write_val(s, (uint32_t)(v >> 32));
        }
        break;
    case SYST_VAL:
        mtk_systimer_write_val(s, (uint32_t)v);
        break;
    default:
        /* Counter registers: free-running, writes ignored. */
        break;
    }
}

static const MemoryRegionOps mtk_systimer_ops = {
    .read = mtk_systimer_read,
    .write = mtk_systimer_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

static void mtk_systimer_reset(DeviceState *dev)
{
    MtkSysTimerState *s = MTK_SYSTIMER(dev);

    mtk_systimer_stop(s);
    s->con = 0;
    s->val = 0;
    s->status = false;
    mtk_systimer_update_irq(s);
}

static void mtk_systimer_realize(DeviceState *dev, Error **errp)
{
    MtkSysTimerState *s = MTK_SYSTIMER(dev);
    const char *log = getenv("MTK_SYSTLOG");

    memory_region_init_io(&s->iomem, OBJECT(dev), &mtk_systimer_ops, s,
                          "mtk-systimer", s->size);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, mtk_systimer_expire, s);
    s->deadline_ns = -1;
    s->log_left = (log && *log && strcmp(log, "0")) ? 64 : 0;
}

static int mtk_systimer_post_load(void *opaque, int version_id)
{
    MtkSysTimerState *s = opaque;

    if (s->deadline_ns >= 0) {
        timer_mod(s->timer, s->deadline_ns);
    } else {
        timer_del(s->timer);
    }
    return 0;
}

static const VMStateDescription vmstate_mtk_systimer = {
    .name = TYPE_MTK_SYSTIMER,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = mtk_systimer_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(con, MtkSysTimerState),
        VMSTATE_UINT32(val, MtkSysTimerState),
        VMSTATE_INT64(deadline_ns, MtkSysTimerState),
        VMSTATE_BOOL(status, MtkSysTimerState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property mtk_systimer_props[] = {
    DEFINE_PROP_UINT32("size", MtkSysTimerState, size, 0x1000),
    DEFINE_PROP_UINT32("freq", MtkSysTimerState, freq_hz, 13000000),
};

static void mtk_systimer_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = mtk_systimer_realize;
    dc->vmsd = &vmstate_mtk_systimer;
    device_class_set_legacy_reset(dc, mtk_systimer_reset);
    device_class_set_props(dc, mtk_systimer_props);
}

static const TypeInfo mtk_systimer_info = {
    .name = TYPE_MTK_SYSTIMER,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MtkSysTimerState),
    .class_init = mtk_systimer_class_init,
};

static void mtk_systimer_register_types(void)
{
    type_register_static(&mtk_systimer_info);
}

type_init(mtk_systimer_register_types)
