/*
 * MediaTek MT6375 Sub-PMIC + Type-C Port Controller (TCPCI) + BC1.2 model.
 *
 * Register layout and semantics verified against xaga vendor modules
 * (mt6375.ko, tcpc_mt6375.ko, mt6375-charger.ko, mt6375-adc.ko), preloader
 * bank table (4e 34 1a 4a 64 3f), and device FDT:
 *  - Bank 0 (0x4e): TCPCI + [0xf2, reg] extended page (VID 0x29cf, PID 0x6375)
 *  - Bank 1 (0x34): PMU/BC1.2/ADC/IRQ (DEV_INFO 0x72, W1C IRQ_EVT 0xd0..0xdf,
 *                   STAT0 0xe0, IRQ_MASK 0xf0..0xff, BC1.2 0x50..0x51, ADC 0xa6)
 *  - Bank 2 (0x1a): GM30 / fuel-gauge efuse helpers (0x58=0x12, 0x59=0x34, 0xae=0x01)
 *  - Bank 3 (0x4a): CHG/ADC register bank
 *  - Bank 4 (0x64): AUXADC register bank (0x11=0x80)
 *  - Bank 5 (0x3f): MISC register bank
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/mtk-env.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/i2c/mt6375.h"

#define MT6375_NUM_BANKS     6
#define MT6375_BC12_DET_MS   130

#define TYPE_MT6375_BANK     "mt6375-bank"
OBJECT_DECLARE_SIMPLE_TYPE(Mt6375BankState, MT6375_BANK)

struct Mt6375State {
    I2CSlave parent_obj;

    qemu_irq irq;
    QEMUTimer *bc12_timer;
    int64_t bc12_start_ms;

    uint8_t banks[MT6375_NUM_BANKS][256];
    uint8_t ext_f2[256];

    uint8_t ptr[MT6375_NUM_BANKS];
    bool first[MT6375_NUM_BANKS];
    bool got_ptr[MT6375_NUM_BANKS];
    bool ext_f2_active;

    bool vbus_present;
    bool trace;
};

struct Mt6375BankState {
    I2CSlave parent_obj;

    Mt6375State *parent;
    uint8_t bank;
};

static const uint8_t mt6375_bank_addrs[MT6375_NUM_BANKS] = {
    0x4e, /* Bank 0: TCPC */
    0x34, /* Bank 1: PMU / BC1.2 / ADC / IRQ (primary TYPE_MT6375 slave) */
    0x1a, /* Bank 2: GM30 */
    0x4a, /* Bank 3: CHG */
    0x64, /* Bank 4: AUXADC */
    0x3f, /* Bank 5: MISC */
};

static G_GNUC_PRINTF(2, 3) void mt6375_log(const Mt6375State *s,
                                          const char *fmt, ...)
{
    va_list ap;

    if (!s->trace) {
        return;
    }
    va_start(ap, fmt);
    fputs("MT6375: ", stderr);
    vfprintf(stderr, fmt, ap);
    fputc('\n', stderr);
    va_end(ap);
}

static void mt6375_update_irq(Mt6375State *s)
{
    bool pending = false;
    unsigned i;

    for (i = 0; i < 16; i++) {
        if (s->banks[1][0xd0 + i] & (uint8_t)~s->banks[1][0xf0 + i]) {
            pending = true;
            break;
        }
    }
    qemu_set_irq(s->irq, pending ? 1 : 0);
}

static uint16_t mt6375_bank0_u16(const Mt6375State *s, uint8_t reg)
{
    return (uint16_t)s->banks[0][reg] | ((uint16_t)s->banks[0][reg + 1] << 8);
}

static void mt6375_put_adc_be16(Mt6375State *s, uint8_t ch, uint16_t raw)
{
    uint8_t reg = (ch == 0) ? 0x9c : (uint8_t)(0xa8 + 2 * ch);

    if ((unsigned)reg + 1 < 0xd0) {
        s->banks[1][reg] = (raw >> 8) & 0xff;
        s->banks[1][reg + 1] = raw & 0xff;
    }
}

static uint16_t mt6375_adc_sample_for_channel(const Mt6375State *s, uint8_t ch)
{
    switch (ch) {
    case 1: /* VBUS / CHGVIN: 6250 uV/LSB -> 800 * 6.25 mV = 5000 mV */
        return s->vbus_present ? 800 : 0;
    case 3: /* VSYS / CHG: 1250 uV/LSB -> 3080 * 1.25 mV = 3850 mV */
    case 4: /* VBAT: 1250 uV/LSB -> 3850 mV */
        return 3080;
    case 8: /* TEMP_JC: offset -56, scale 1 -> 25 C */
        return 81;
    case 14: /* SBU1 water-detect: 2500 uV/LSB; pull-up (5) = 1200 mV, else 0 */
        return (s->banks[0][0xd2] == 5) ? 480 : 0;
    case 15: /* SBU2 water-detect */
        return (s->banks[0][0xd4] == 5) ? 480 : 0;
    default:
        return 2000;
    }
}

static void mt6375_refresh_all_adc(Mt6375State *s)
{
    uint8_t ch;

    for (ch = 0; ch <= 16; ch++) {
        mt6375_put_adc_be16(s, ch, mt6375_adc_sample_for_channel(s, ch));
    }
}

static void mt6375_reset_bank0(Mt6375State *s)
{
    memset(s->banks[0], 0, 256);
    /* TCPCI standard identity registers (LE16): VID 0x29cf, PID 0x6375, DID 0x0002 */
    s->banks[0][0x00] = 0xcf;
    s->banks[0][0x01] = 0x29;
    s->banks[0][0x02] = 0x75;
    s->banks[0][0x03] = 0x63;
    s->banks[0][0x04] = 0x02;
    s->banks[0][0x05] = 0x00;

    s->banks[0][0x10] = s->vbus_present ? 0x03 : 0x00; /* ALERT: CC | POWER_STATUS */
    s->banks[0][0x11] = 0x00;
    s->banks[0][0x1d] = s->vbus_present ? 0x11 : 0x00; /* CC_STATUS: Rd + CC1 Rp-Default */
    s->banks[0][0x1e] = s->vbus_present ? 0x0c : 0x00; /* POWER_STATUS: VBUS present */
    s->banks[0][0x9f] = s->vbus_present ? 0x00 : 0x02; /* RT_STATUS: VSAFE0V when unplugged */
}

static void mt6375_bc12_complete(void *opaque)
{
    Mt6375State *s = MT6375(opaque);

    if (!s->vbus_present || !(s->banks[1][0x50] & 0x80)) {
        return;
    }
    s->banks[1][0x51] = (s->banks[1][0x51] & 0xf0) | 0x0d; /* PORT_STAT = SDP */
    s->banks[1][0xd3] |= 0x80; /* fl_bc12_dn (IRQ 0x1f) */
    mt6375_log(s, "BC1.2 done -> SDP (0x151=%#x)", s->banks[1][0x51]);
    mt6375_update_irq(s);
}

static void mt6375_bc12_sync(Mt6375State *s)
{
    if (!(s->banks[1][0x50] & 0x80) || s->bc12_start_ms < 0 || !s->vbus_present) {
        return;
    }
    if (qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) - s->bc12_start_ms >= MT6375_BC12_DET_MS) {
        s->banks[1][0x51] = (s->banks[1][0x51] & 0xf0) | 0x0d;
    }
}

static uint8_t mt6375_bank_read(Mt6375State *s, uint8_t bank, uint8_t reg)
{
    uint8_t val = s->banks[bank][reg];

    if (bank == 0) {
        switch (reg) {
        case 0x1d:
            val = s->vbus_present ? 0x11 : 0x00;
            break;
        case 0x1e:
            val = s->vbus_present ? 0x0c : 0x00;
            break;
        case 0x9f:
            val = s->vbus_present ? 0x00 : 0x02;
            break;
        default:
            break;
        }
    } else if (bank == 1) {
        switch (reg) {
        case 0x00:
            val = 0x72;
            break;
        case 0x51:
            mt6375_bc12_sync(s);
            val = s->banks[1][0x51];
            break;
        case 0xe0:
            val = (s->banks[1][0xe0] & ~0x01u) | (s->vbus_present ? 0x01u : 0x00u);
            break;
        default:
            break;
        }
    }
    mt6375_log(s, "R bank%u[0x%02x] -> 0x%02x", bank, reg, val);
    return val;
}

static void mt6375_bank_write(Mt6375State *s, uint8_t bank, uint8_t reg,
                              uint8_t val)
{
    mt6375_log(s, "W bank%u[0x%02x] <- 0x%02x", bank, reg, val);

    if (bank == 0) {
        switch (reg) {
        case 0x00 ... 0x05:
        case 0x1d:
        case 0x1e:
        case 0x9f:
            return; /* read-only ID and live status registers */
        case 0x10:
        case 0x11:
            s->banks[0][reg] &= ~val; /* ALERT is W1C */
            return;
        case 0x12:
        case 0x13: {
            uint16_t old_mask = mt6375_bank0_u16(s, 0x12);
            uint16_t new_mask;
            uint16_t alert = mt6375_bank0_u16(s, 0x10);

            s->banks[0][reg] = val;
            new_mask = mt6375_bank0_u16(s, 0x12);
            if ((new_mask & ~old_mask & alert) != 0) {
                s->banks[1][0xdf] |= 0x01; /* pd_evt (IRQ 0x78) */
                mt6375_update_irq(s);
            }
            return;
        }
        case 0x23: /* TCPC_V10_REG_COMMAND */
            s->banks[0][reg] = val;
            if (val == 0x99 && s->vbus_present) {
                /*
                 * TCPM_CMD_LOOK_CONNECTION: guest TCPCI driver restarted
                 * Type-C DRP toggling after probe; re-latch CC/power-status
                 * alerts and the PD summary event bit so the Type-C state
                 * machine observes the attached sink/VBUS state.
                 */
                s->banks[0][0x10] |= 0x03;
                s->banks[1][0xdf] |= 0x01;
                mt6375_update_irq(s);
            }
            return;
        case 0xb0: /* SWRESET */
            s->banks[0][reg] = val;
            if (val & 0x01) {
                mt6375_reset_bank0(s);
                mt6375_update_irq(s);
            }
            return;
        default:
            s->banks[0][reg] = val;
            if (reg == 0xd2 || reg == 0xd4) {
                mt6375_refresh_all_adc(s);
            }
            return;
        }
    }

    if (bank == 1) {
        switch (reg) {
        case 0x00:
            return; /* DEV_INFO read-only */
        case 0x50: {
            uint8_t old = s->banks[1][0x50];

            s->banks[1][0x50] = val;
            if (!(old & 0x80) && (val & 0x80) &&
                s->vbus_present && !MTK_ENV_CACHED("MTK_NO_BC12")) {
                s->bc12_start_ms = qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL);
                timer_mod(s->bc12_timer,
                          s->bc12_start_ms + MT6375_BC12_DET_MS);
            } else if (!(val & 0x80) || !s->vbus_present) {
                s->bc12_start_ms = -1;
                timer_del(s->bc12_timer);
                s->banks[1][0x51] &= ~0x0fu;
            }
            return;
        }
        case 0xa6: {
            uint8_t ch = (val >> 4) & 0x0f;

            mt6375_refresh_all_adc(s);
            mt6375_put_adc_be16(s, ch, mt6375_adc_sample_for_channel(s, ch));
            s->banks[1][0xa6] = 0x00; /* self-clear trigger */
            s->banks[1][0xd5] |= 0x10; /* adc_donei (IRQ 0x2c) */
            mt6375_update_irq(s);
            return;
        }
        case 0xd0 ... 0xdf:
            s->banks[1][reg] &= ~val; /* IRQ_EVT is W1C */
            mt6375_update_irq(s);
            return;
        case 0xe0:
            s->banks[1][0xe0] = (val & ~0x01u) | (s->vbus_present ? 0x01u : 0x00u);
            return;
        case 0xf0 ... 0xff: {
            uint8_t old = s->banks[1][reg];

            s->banks[1][reg] = val;
            if (reg == 0xf0 && (old & 0x01) && !(val & 0x01) && s->vbus_present) {
                s->banks[1][0xd0] |= 0x01; /* fl_pwr_rdy (IRQ 0x00) */
            }
            if (reg == 0xff && (old & 0x01) && !(val & 0x01)) {
                uint16_t alert = mt6375_bank0_u16(s, 0x10);
                uint16_t mask = mt6375_bank0_u16(s, 0x12);

                if ((alert & mask) != 0) {
                    s->banks[1][0xdf] |= 0x01; /* pd_evt (IRQ 0x78) */
                }
            }
            mt6375_update_irq(s);
            return;
        }
        default:
            s->banks[1][reg] = val;
            return;
        }
    }

    s->banks[bank][reg] = val;
}

static int mt6375_bus_event(Mt6375State *s, uint8_t bank, enum i2c_event ev)
{
    if (ev == I2C_START_SEND) {
        s->first[bank] = true;
        s->got_ptr[bank] = false;
        if (bank == 0) {
            s->ext_f2_active = false;
        }
    } else if (ev == I2C_FINISH) {
        s->first[bank] = false;
    }
    return 0;
}

static int mt6375_bus_send(Mt6375State *s, uint8_t bank, uint8_t data)
{
    if (bank == 0) {
        if (s->first[0]) {
            s->first[0] = false;
            if (data == 0xf2) {
                s->ext_f2_active = true;
                s->got_ptr[0] = false;
            } else {
                s->ext_f2_active = false;
                s->ptr[0] = data;
                s->got_ptr[0] = true;
            }
        } else if (!s->got_ptr[0]) {
            s->ptr[0] = data;
            s->got_ptr[0] = true;
        } else if (s->ext_f2_active) {
            mt6375_log(s, "W ext_f2[0x%02x] <- 0x%02x", s->ptr[0], data);
            s->ext_f2[s->ptr[0]++] = data;
        } else {
            mt6375_bank_write(s, 0, s->ptr[0]++, data);
        }
        return 0;
    }

    if (s->first[bank]) {
        s->first[bank] = false;
        s->ptr[bank] = data;
        s->got_ptr[bank] = true;
    } else {
        mt6375_bank_write(s, bank, s->ptr[bank]++, data);
    }
    return 0;
}

static uint8_t mt6375_bus_recv(Mt6375State *s, uint8_t bank)
{
    if (bank == 0 && s->ext_f2_active) {
        if (s->got_ptr[0]) {
            uint8_t reg = s->ptr[0]++;
            uint8_t val = s->ext_f2[reg];

            mt6375_log(s, "R ext_f2[0x%02x] -> 0x%02x", reg, val);
            return val;
        }
        s->ext_f2_active = false;
        s->ptr[0] = 0xf2;
    }
    return mt6375_bank_read(s, bank, s->ptr[bank]++);
}

static int mt6375_event(I2CSlave *i2c, enum i2c_event ev)
{
    return mt6375_bus_event(MT6375(i2c), 1, ev);
}

static int mt6375_send(I2CSlave *i2c, uint8_t data)
{
    return mt6375_bus_send(MT6375(i2c), 1, data);
}

static uint8_t mt6375_recv(I2CSlave *i2c)
{
    return mt6375_bus_recv(MT6375(i2c), 1);
}

static void mt6375_reset(DeviceState *dev)
{
    Mt6375State *s = MT6375(dev);

    if (s->bc12_timer) {
        timer_del(s->bc12_timer);
    }
    s->bc12_start_ms = -1;

    memset(s->banks, 0, sizeof(s->banks));
    memset(s->ext_f2, 0, sizeof(s->ext_f2));
    memset(s->ptr, 0, sizeof(s->ptr));
    memset(s->first, 0, sizeof(s->first));
    memset(s->got_ptr, 0, sizeof(s->got_ptr));
    s->ext_f2_active = false;

    /* Bank 0: TCPCI defaults */
    mt6375_reset_bank0(s);

    /* Bank 1: PMU / BC1.2 / IRQ defaults */
    s->banks[1][0x00] = 0x72; /* DEV_INFO (rev 0x2) */
    s->banks[1][0xe0] = s->vbus_present ? 0x01 : 0x00; /* STAT0: F_ST_PWR_RDY */
    memset(&s->banks[1][0xf0], 0xff, 16); /* IRQ_MASK: all masked at reset */
    mt6375_refresh_all_adc(s);

    /* Bank 2: GM30 / efuse helpers */
    s->banks[2][0x58] = 0x12;
    s->banks[2][0x59] = 0x34;
    s->banks[2][0xae] = 0x01;

    /* Bank 4: AUXADC */
    s->banks[4][0x11] = 0x80;

    mt6375_update_irq(s);
}

static void mt6375_realize(DeviceState *dev, Error **errp)
{
    Mt6375State *s = MT6375(dev);

    if (MTK_ENV_CACHED("MTK_MT6375_NO_VBUS")) {
        s->vbus_present = false;
    }
    if (MTK_ENV_CACHED("MTK_MT6375_LOG")) {
        s->trace = true;
    }
    qdev_init_gpio_out(dev, &s->irq, 1);
    s->bc12_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, mt6375_bc12_complete, s);
}

static void mt6375_unrealize(DeviceState *dev)
{
    Mt6375State *s = MT6375(dev);

    if (s->bc12_timer) {
        timer_free(s->bc12_timer);
        s->bc12_timer = NULL;
    }
}

static const Property mt6375_props[] = {
    DEFINE_PROP_BOOL("vbus-present", Mt6375State, vbus_present, true),
    DEFINE_PROP_BOOL("trace", Mt6375State, trace, false),
};

static void mt6375_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->event = mt6375_event;
    k->send = mt6375_send;
    k->recv = mt6375_recv;
    dc->realize = mt6375_realize;
    dc->unrealize = mt6375_unrealize;
    device_class_set_legacy_reset(dc, mt6375_reset);
    device_class_set_props(dc, mt6375_props);
    dc->desc = "MediaTek MT6375 sub-PMIC, TCPC and BC1.2 controller";
}

static int mt6375_bank_event(I2CSlave *i2c, enum i2c_event ev)
{
    Mt6375BankState *b = MT6375_BANK(i2c);

    g_assert(b->parent && b->bank < MT6375_NUM_BANKS);
    return mt6375_bus_event(b->parent, b->bank, ev);
}

static int mt6375_bank_send(I2CSlave *i2c, uint8_t data)
{
    Mt6375BankState *b = MT6375_BANK(i2c);

    g_assert(b->parent && b->bank < MT6375_NUM_BANKS);
    return mt6375_bus_send(b->parent, b->bank, data);
}

static uint8_t mt6375_bank_recv(I2CSlave *i2c)
{
    Mt6375BankState *b = MT6375_BANK(i2c);

    g_assert(b->parent && b->bank < MT6375_NUM_BANKS);
    return mt6375_bus_recv(b->parent, b->bank);
}

static const Property mt6375_bank_props[] = {
    DEFINE_PROP_LINK("parent", Mt6375BankState, parent,
                     TYPE_MT6375, Mt6375State *),
    DEFINE_PROP_UINT8("bank", Mt6375BankState, bank, 0),
};

static void mt6375_bank_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->event = mt6375_bank_event;
    k->send = mt6375_bank_send;
    k->recv = mt6375_bank_recv;
    device_class_set_props(dc, mt6375_bank_props);
    dc->desc = "MediaTek MT6375 I2C bank proxy slave";
}

DeviceState *mt6375_attach(I2CBus *bus, qemu_irq irq)
{
    DeviceState *dev = DEVICE(i2c_slave_new(TYPE_MT6375, mt6375_bank_addrs[1]));
    uint8_t b;

    i2c_slave_realize_and_unref(I2C_SLAVE(dev), bus, &error_fatal);
    if (irq) {
        qdev_connect_gpio_out(dev, 0, irq);
    }

    for (b = 0; b < MT6375_NUM_BANKS; b++) {
        DeviceState *proxy;

        if (b == 1) {
            continue;
        }
        proxy = DEVICE(i2c_slave_new(TYPE_MT6375_BANK, mt6375_bank_addrs[b]));
        object_property_set_link(OBJECT(proxy), "parent", OBJECT(dev),
                                 &error_fatal);
        qdev_prop_set_uint8(proxy, "bank", b);
        i2c_slave_realize_and_unref(I2C_SLAVE(proxy), bus, &error_fatal);
    }
    return dev;
}

static const TypeInfo mt6375_types[] = {
    {
        .name = TYPE_MT6375,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(Mt6375State),
        .class_init = mt6375_class_init,
    }, {
        .name = TYPE_MT6375_BANK,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(Mt6375BankState),
        .class_init = mt6375_bank_class_init,
    },
};

DEFINE_TYPES(mt6375_types)
