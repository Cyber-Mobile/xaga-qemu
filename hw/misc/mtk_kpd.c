/*
 * MediaTek KPD (keypad controller) model for the MT6895 machine.
 *
 * The vendor driver (vendor module analysis, not stripped) is
 * bound to 1c00e000.kp on this machine.  Disassembling kpd_get_keymap_state
 * (0x91c) and kpd_keymap_handler (0x700) gives the register contract:
 *
 *   base + 0x04 .. 0x14  five 16-bit KP_MEM1..KP_MEM5 state words, read with
 *                        readw() whenever the keypad tasklet runs.  Bit j of
 *                        word i is hardware key i*16 + j; the device tree's
 *                        hw-init-map[0] = 114 (KEY_VOLUMEDOWN) makes hardware
 *                        key 0 the volume-down key.
 *
 *   The state is ACTIVE LOW.  kpd_pdrv_probe initialises the driver's cached
 *   keymap to 0xffffffff ("all released"), and kpd_keymap_handler calls
 *   input_event(EV_KEY, code, !(change_bit & new_state)), so a cleared bit
 *   means the key is held and a set bit means it is released.
 *
 *   LK uses the same layout with the xaga DT's hardware key numbers:
 *   kpd_hw_pwrkey = 8 (POWER), kpd_hw_rstkey = 17 (KEY_VOLUMEUP),
 *   kpd_hw_recovery_key = 17.  For example volume-up = hardware key 17 =
 *   KP_MEM2 bit 1 (word at +0x08).
 *
 *   IRQ: GIC SPI 106, IRQ_TYPE_EDGE_RISING in the device tree.  The kernel
 *   handler kpd_irq_handler (0x8a0) does nothing but disable_irq_nosync() and
 *   tasklet_schedule(), so the model pulses the line on any key-state change
 *   and the tasklet then reads the five words.
 *
 *   The driver also writes base+0x18 (debounce) and base+0x20 (enable) during
 *   probe.  Those (and every other register) are kept in a plain register
 *   file; the model needs no behaviour from them.
 *
 * Host input: a QEMU input handler is registered, so HMP
 * "sendkey volumedown", "sendkey volumeup"/"volumeup" and "sendkey power"
 * (and a real keyboard once a display backend such as VNC is enabled) drive
 * the key state.
 *
 * Boot keys (for the LK boot-mode menu / Recovery):
 *   MTK_KPD_BOOT_KEYS=volup[,voldown][,power]  (or raw hardware key indexes)
 *       press these keys from machine reset, i.e. before LK samples the
 *       keypad, which is what LK's get_kpd_dts_info/boot-mode path reads.
 *   MTK_KPD_BOOT_HWKEY=<n>[,<n>...]   same, raw hardware key indexes.
 *   MTK_KPD_HOLD=<ms>                 how long to keep them held (default
 *                                     20000).  The state is released by a
 *                                     virtual timer and the release pulses
 *                                     the IRQ.
 *   MTK_KPD_PULSE_MS=<ms>             while held, pulse the IRQ line every
 *                                     <ms> (default 100) so a driver that
 *                                     enables its IRQ late still sees it.
 *
 * Env:
 *   MTK_KPDLOG=1  trace register reads/writes, key transitions and boot-key
 *                 arming on stderr, with host realtime timestamps.
 *   MTK_NO_KPD=1  handled by the machine: the old flat register window is
 *                 installed instead of this device.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/mtk_spmi_pmif.h"
#include "qom/object.h"
#include "ui/input.h"

#define TYPE_MTK_KPD "mtk-kpd"
OBJECT_DECLARE_SIMPLE_TYPE(MtkKpdState, MTK_KPD)

#define MTK_KPD_SIZE            0x1000
#define MTK_KPD_MEM_OFF         0x04
#define MTK_KPD_MEM_NUM         5
#define MTK_KPD_MEM_STRIDE      4
#define MTK_KPD_KEY_RELEASED    0xffff
#define MTK_KPD_NUM_BITS        (MTK_KPD_MEM_NUM * 16)
#define MTK_KPD_MAX_BOOT_KEYS   8

struct MtkKpdState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    /* Plain register file for everything except the key-state words. */
    uint8_t regs[MTK_KPD_SIZE];
    /* KP_MEM1..KP_MEM5, active low. */
    uint16_t mem[MTK_KPD_MEM_NUM];

    bool log;
    QemuInputHandlerState *input;

    /* MT6363 PMIC key block that receives the power key edges. */
    MtkSpmiPmifState *pmic;

    /* Boot keys held from reset until MTK_KPD_HOLD expires. */
    bool boot_keys;
    uint16_t boot_mask[MTK_KPD_MEM_NUM];
    QEMUTimer *pulse_timer;
    QEMUTimer *hold_timer;
    QEMUTimer *cycle_timer;
    int64_t pulse_period_ns;
    int64_t cycle_period_ns;
    int64_t boot_hold_delay_ns;
    bool boot_cycle;
    bool boot_pressed;
};

static double mtk_kpd_now(MtkKpdState *s)
{
    return qemu_clock_get_ns(QEMU_CLOCK_REALTIME) / 1e9;
}

static bool mtk_kpd_is_mem(hwaddr off)
{
    return off >= MTK_KPD_MEM_OFF &&
           off < MTK_KPD_MEM_OFF + MTK_KPD_MEM_NUM * MTK_KPD_MEM_STRIDE;
}

static uint8_t mtk_kpd_byte(MtkKpdState *s, hwaddr off)
{
    unsigned idx;

    if (mtk_kpd_is_mem(off)) {
        idx = (off - MTK_KPD_MEM_OFF) / MTK_KPD_MEM_STRIDE;
        return (s->mem[idx] >>
                (((off - MTK_KPD_MEM_OFF) % MTK_KPD_MEM_STRIDE) * 8)) & 0xff;
    }
    return s->regs[off];
}

static uint64_t mtk_kpd_read(void *opaque, hwaddr off, unsigned size)
{
    MtkKpdState *s = opaque;
    uint64_t val = 0;
    unsigned i;

    if (off >= MTK_KPD_SIZE || size > MTK_KPD_SIZE - off) {
        return 0;
    }

    /* DEVICE_LITTLE_ENDIAN: assemble the bytes LSB first. */
    for (i = 0; i < size; i++) {
        val |= (uint64_t)mtk_kpd_byte(s, off + i) << (8 * i);
    }

    if (s->log) {
        fprintf(stderr, "MTK-KPD [%9.3f] read  +0x%03x/%u -> 0x%0*" PRIx64 "\n",
                mtk_kpd_now(s), (unsigned)off, size, (int)(size * 2), val);
    }
    return val;
}

static void mtk_kpd_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    MtkKpdState *s = opaque;
    unsigned i;

    if (off >= MTK_KPD_SIZE || size > MTK_KPD_SIZE - off) {
        return;
    }

    if (s->log) {
        fprintf(stderr, "MTK-KPD [%9.3f] write +0x%03x/%u <- 0x%0*" PRIx64 "%s\n",
                mtk_kpd_now(s), (unsigned)off, size, (int)(size * 2), val,
                mtk_kpd_is_mem(off) ? " (key state is read-only, ignored)" : "");
    }
    if (mtk_kpd_is_mem(off)) {
        return;
    }

    for (i = 0; i < size; i++) {
        s->regs[off + i] = (val >> (8 * i)) & 0xff;
    }
}

static const MemoryRegionOps mtk_kpd_ops = {
    .read = mtk_kpd_read,
    .write = mtk_kpd_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
    .impl = { .min_access_size = 1, .max_access_size = 8 },
};

static int mtk_kpd_hwkey_from_name(const char *name)
{
    if (!strcmp(name, "volup")) {
        return 17;              /* DT kpd_hw_rstkey / kpd_hw_recovery_key */
    }
    if (!strcmp(name, "voldown")) {
        return 0;               /* DT hw-init-map[0] = 114 KEY_VOLUMEDOWN */
    }
    if (!strcmp(name, "power")) {
        return 8;               /* DT kpd_hw_pwrkey */
    }
    return -1;
}

static void mtk_kpd_pulse_cb(void *opaque)
{
    MtkKpdState *s = opaque;

    qemu_irq_pulse(s->irq);
    if (s->boot_keys && s->pulse_timer) {
        timer_mod(s->pulse_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->pulse_period_ns);
    }
}

static void mtk_kpd_hold_cb(void *opaque)
{
    MtkKpdState *s = opaque;
    unsigned i;

    if (!s->boot_keys) {
        return;
    }
    if (s->pulse_timer) {
        timer_del(s->pulse_timer);
    }
    if (s->cycle_timer) {
        timer_del(s->cycle_timer);
    }
    s->boot_keys = false;
    s->boot_cycle = false;
    for (i = 0; i < MTK_KPD_MEM_NUM; i++) {
        s->mem[i] |= s->boot_mask[i];
    }
    if (s->log) {
        fprintf(stderr, "MTK-KPD [%9.3f] boot keys released\n",
                mtk_kpd_now(s));
    }
    qemu_irq_pulse(s->irq);
}

/*
 * Press the boot keys configured through the environment and arm the release
 * (and repeated IRQ pulse) timers.  Called from reset and, in case the reset
 * runs before the timers exist, from realize as well.  Idempotent.
 */
static void mtk_kpd_arm_boot_keys(MtkKpdState *s)
{
    const char *env, *es;
    char *copy, *tok, *save = NULL;
    unsigned nbits = 0;
    int64_t hold_ns;
    unsigned i;

    s->boot_keys = false;
    memset(s->boot_mask, 0, sizeof(s->boot_mask));

    env = getenv("MTK_KPD_BOOT_KEYS");
    if (!env || !*env) {
        env = getenv("MTK_KPD_BOOT_HWKEY");
    }
    if (!env || !*env) {
        return;
    }

    copy = g_strdup(env);
    for (tok = strtok_r(copy, ", ", &save); tok;
         tok = strtok_r(NULL, ", ", &save)) {
        int hw;

        if (tok[0] >= '0' && tok[0] <= '9') {
            hw = atoi(tok);
        } else {
            hw = mtk_kpd_hwkey_from_name(tok);
        }
        if (hw < 0 || hw >= MTK_KPD_NUM_BITS) {
            fprintf(stderr,
                    "MTK-KPD: unknown boot key / hw index '%s' (use volup, "
                    "voldown, power or a hardware key index 0..%d)\n",
                    tok, MTK_KPD_NUM_BITS - 1);
            continue;
        }
        s->boot_mask[hw / 16] |= 1u << (hw % 16);
        nbits++;
    }
    g_free(copy);
    if (!nbits) {
        return;
    }

    s->boot_keys = true;
    for (i = 0; i < MTK_KPD_MEM_NUM; i++) {
        s->mem[i] &= ~s->boot_mask[i];
    }

    es = getenv("MTK_KPD_HOLD");
    hold_ns = (es && *es ? atoi(es) : 20000) * 1000000LL;
    es = getenv("MTK_KPD_PULSE_MS");
    s->pulse_period_ns = (es && *es ? atoi(es) : 100) * 1000000LL;
    if (s->pulse_period_ns <= 0) {
        s->pulse_period_ns = 100 * 1000000LL;
    }
    es = getenv("MTK_KPD_BOOT_CYCLE");
    s->cycle_period_ns = (es && *es ? atoi(es) : 0) * 1000000LL;
    s->boot_cycle = s->cycle_period_ns > 0;
    s->boot_pressed = false;
    es = getenv("MTK_KPD_BOOT_DELAY");
    s->boot_hold_delay_ns = (es && *es ? atoi(es) : 0) * 1000000LL;

    if (s->boot_cycle) {
        /* Leave the state released until the delay expires. */
        for (i = 0; i < MTK_KPD_MEM_NUM; i++) {
            s->mem[i] |= s->boot_mask[i];
        }
    }

    if (s->log) {
        fprintf(stderr,
                "MTK-KPD [%9.3f] boot keys held: mask MEM1..5 ="
                " %04x %04x %04x %04x %04x, hold %" PRId64 " ms\n",
                mtk_kpd_now(s), s->boot_mask[0], s->boot_mask[1],
                s->boot_mask[2], s->boot_mask[3], s->boot_mask[4],
                hold_ns / 1000000);
    }

    if (s->pulse_timer && s->hold_timer) {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

        if (s->boot_cycle && s->cycle_timer) {
            timer_mod(s->cycle_timer, now + s->boot_hold_delay_ns);
        } else {
            timer_mod(s->pulse_timer, now + 1000000);
        }
        timer_mod(s->hold_timer, now + hold_ns + s->boot_hold_delay_ns);
    }
}

/*
 * Boot-key cycling (MTK_KPD_BOOT_CYCLE=<ms>): alternate pressed/released while
 * the boot-key window is open, so a driver that binds long after reset (the
 * kernel's mtk-kpd.ko, or recovery's evdev consumer) keeps receiving real
 * press/release edges.  MTK_KPD_BOOT_DELAY=<ms> postpones the first edge so
 * the early boot stages (LK) do not see the test key.
 */
static void mtk_kpd_cycle_cb(void *opaque)
{
    MtkKpdState *s = opaque;
    unsigned i;

    if (!s->boot_keys || !s->boot_cycle) {
        return;
    }
    s->boot_pressed = !s->boot_pressed;
    for (i = 0; i < MTK_KPD_MEM_NUM; i++) {
        if (s->boot_pressed) {
            s->mem[i] &= ~s->boot_mask[i];
        } else {
            s->mem[i] |= s->boot_mask[i];
        }
    }
    if (s->log) {
        fprintf(stderr, "MTK-KPD [%9.3f] boot-cycle %s, MEM1..5 ="
                " %04x %04x %04x %04x %04x\n",
                mtk_kpd_now(s), s->boot_pressed ? "press" : "release",
                s->mem[0], s->mem[1], s->mem[2], s->mem[3], s->mem[4]);
    }
    qemu_irq_pulse(s->irq);
    timer_mod(s->cycle_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->cycle_period_ns);
}

/*
 * Apply one host key transition to the active-low key-state words and pulse
 * the interrupt line.  The driver disables the IRQ while its tasklet runs, so
 * an edge per change is enough; it re-reads the state in the tasklet.
 */
static void mtk_kpd_key_change(MtkKpdState *s, unsigned bit, bool down)
{
    unsigned mem, b;
    uint16_t old;

    if (bit >= MTK_KPD_NUM_BITS) {
        return;
    }
    mem = bit / 16;
    b = bit % 16;
    old = s->mem[mem];

    if (down) {
        s->mem[mem] &= ~(1u << b);
    } else {
        s->mem[mem] |= 1u << b;
    }
    if (s->mem[mem] == old) {
        return;
    }

    if (s->log) {
        fprintf(stderr,
                "MTK-KPD [%9.3f] %s hw-key %u (KP_MEM%u bit%u) -> 0x%04x,"
                " pulse irq\n",
                mtk_kpd_now(s), down ? "press  " : "release", bit, mem + 1, b,
                s->mem[mem]);
    }
    qemu_irq_pulse(s->irq);
}

static void mtk_kpd_input_event(DeviceState *dev, QemuConsole *src,
                                QemuInputEvent *evt)
{
    MtkKpdState *s = MTK_KPD(dev);
    unsigned bit;

    if (evt->type != INPUT_EVENT_KIND_KEY) {
        return;
    }

    /*
     * The QEMU input core delivers key events to handlers as Linux keycodes
     * (qmp_input_send_event converts with qemu_input_key_value_to_linux, and
     * qemu_input_event_send_key_linux passes the code through as-is), so
     * convert before matching.  This one path then covers HMP sendkey, QMP
     * input-send-event and VNC/GTK keyboards.
     */
    switch (qemu_input_linux_to_qcode(evt->key.key)) {
    case Q_KEY_CODE_VOLUMEDOWN:     /* hardware key 0: hw-init-map[0]=114 */
        bit = 0;
        break;
    case Q_KEY_CODE_VOLUMEUP:       /* hardware key 17 (LK kpd_hw_rstkey) */
        bit = 17;
        break;
    case Q_KEY_CODE_POWER:          /* hardware key 8 (LK kpd_hw_pwrkey) */
        bit = 8;
        /*
         * On this board the power key is wired to the MT6363 PMIC key block,
         * not to the keypad matrix: forward the edge so the PMIC raises its
         * PWRKEY / PWRKEY_R interrupt and the kernel PMIC key driver reports
         * KEY_POWER.  The keypad state bit is still updated because the
         * bootloader samples it (LK kpd_hw_pwrkey = 8).
         */
        if (s->pmic) {
            mtk_spmi_pmif_power_key(DEVICE(s->pmic), evt->key.down);
        }
        break;
    default:
        return;
    }

    mtk_kpd_key_change(s, bit, evt->key.down);
}

static const QemuInputHandler mtk_kpd_input_handler = {
    .name = "mtk-kpd",
    .mask = INPUT_EVENT_MASK_KEY,
    .event = mtk_kpd_input_event,
};

static void mtk_kpd_reset(DeviceState *dev)
{
    MtkKpdState *s = MTK_KPD(dev);
    unsigned i;

    if (s->pulse_timer) {
        timer_del(s->pulse_timer);
    }
    if (s->hold_timer) {
        timer_del(s->hold_timer);
    }
    memset(s->regs, 0, sizeof(s->regs));
    for (i = 0; i < MTK_KPD_MEM_NUM; i++) {
        s->mem[i] = MTK_KPD_KEY_RELEASED;
    }
    mtk_kpd_arm_boot_keys(s);
}

static void mtk_kpd_init(Object *obj)
{
    MtkKpdState *s = MTK_KPD(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &mtk_kpd_ops, s, TYPE_MTK_KPD,
                          MTK_KPD_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    s->log = getenv("MTK_KPDLOG") != NULL;
    memset(s->mem, 0xff, sizeof(s->mem));
}

static void mtk_kpd_realize(DeviceState *dev, Error **errp)
{
    MtkKpdState *s = MTK_KPD(dev);

    s->pulse_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, mtk_kpd_pulse_cb, s);
    s->hold_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, mtk_kpd_hold_cb, s);
    s->cycle_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, mtk_kpd_cycle_cb, s);

    s->input = qemu_input_handler_register(dev, &mtk_kpd_input_handler);
    qemu_input_handler_activate(s->input);

    /* The timers exist now; re-arm in case reset ran earlier. */
    if (s->boot_keys) {
        mtk_kpd_arm_boot_keys(s);
    }
}

static void mtk_kpd_unrealize(DeviceState *dev)
{
    MtkKpdState *s = MTK_KPD(dev);

    if (s->input) {
        qemu_input_handler_unregister(s->input);
        s->input = NULL;
    }
    if (s->pulse_timer) {
        timer_del(s->pulse_timer);
        timer_free(s->pulse_timer);
        s->pulse_timer = NULL;
    }
    if (s->hold_timer) {
        timer_del(s->hold_timer);
        timer_free(s->hold_timer);
        s->hold_timer = NULL;
    }
    if (s->cycle_timer) {
        timer_del(s->cycle_timer);
        timer_free(s->cycle_timer);
        s->cycle_timer = NULL;
    }
}

static void mtk_kpd_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    static const Property props[] = {
        DEFINE_PROP_LINK("pmic", MtkKpdState, pmic, TYPE_MTK_SPMI_PMIF,
                         MtkSpmiPmifState *),
    };

    device_class_set_legacy_reset(dc, mtk_kpd_reset);
    dc->realize = mtk_kpd_realize;
    dc->unrealize = mtk_kpd_unrealize;
    device_class_set_props(dc, props);
}

static const TypeInfo mtk_kpd_info = {
    .name = TYPE_MTK_KPD,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MtkKpdState),
    .instance_init = mtk_kpd_init,
    .class_init = mtk_kpd_class_init,
};

static void mtk_kpd_register_types(void)
{
    type_register_static(&mtk_kpd_info);
}

type_init(mtk_kpd_register_types)
