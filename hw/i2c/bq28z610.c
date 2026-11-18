/*
 * TI bq28z610 battery fuel gauge (SBS data + ManufacturerAccess mailbox).
 *
 * Register contract verified against the vendor kernel driver
 * (firmware/ref/xaga-kernel/drivers/power/supply/mediatek/bq28z610.c, h
 * bq28z610.h) and the cross-checked notes in notes_peri_spec.md:
 *
 *  - regmap is 8-bit register / 8-bit value.  A "word" register is two
 *    consecutive 8-bit registers read separately (0x08 low byte, 0x09 high
 *    byte), which is how both the bootloader and the kernel driver access it.
 *  - MAC read: write command low byte to ALT_MAC (0x3E), write 0 to 0x3F,
 *    sleep 4 ms, then read 36 bytes as 36 separate byte reads at
 *    0x3E..0x61.  Layout: [0:1] command echo, [2:33] data, [34] checksum,
 *    [35] total length; the driver verifies the checksum over the first
 *    (length - 2) bytes.
 *  - MAC write: command + data written from 0x3E, checksum to 0x60 and
 *    length+4 to 0x61 (accepted, not needed for the probe path).
 *  - Probed identities: DEVICE_NAME "bq28z610", MANU_NAME "MI*" prefix,
 *    DEVICE_CHEM data[1] in {L,F,J}.
 *
 * The battery condition is a modelled physical input, exposed as device
 * properties with a documented default: a healthy pack at 3.85 V, 55 % SOC,
 * 25 C (0.1 K units 2981), 0.5 A discharge, 5000 mAh design/full capacity and
 * 10 cycles.  The vendor DT does not carry the pack capacity, so 5000 mAh is
 * the documented default.  Nothing about the guest's charger decisions is
 * faked: the stack sees this condition and may charge, refuse or shut down.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/i2c/bq28z610.h"
#include "hw/core/qdev-properties.h"

#define SBS_CONTROL      0x00
#define SBS_AT_RATE      0x02
#define SBS_TEMPERATURE  0x06
#define SBS_VOLTAGE      0x08
#define SBS_FLAGS        0x0A
#define SBS_CURRENT      0x0C
#define SBS_AVG_CURRENT  0x14
#define SBS_TTE          0x16
#define SBS_TTF          0x18
#define SBS_REMAINING    0x10
#define SBS_FULL         0x12
#define SBS_CYCLES       0x2A
#define SBS_SOC          0x2C
#define SBS_SOH          0x2E
#define SBS_CHG_VOLTAGE  0x30
#define SBS_CHG_CURRENT  0x32
#define SBS_DESIGN_CAP   0x3C
#define SBS_ALT_MAC      0x3E
#define SBS_MAC_DATA     0x40
#define SBS_MAC_CHKSUM   0x60
#define SBS_MAC_LEN      0x61

/* MAC commands used by the probe / monitor path. */
#define MAC_DEVICE_NAME  0x4A
#define MAC_MANU_NAME    0x4C
#define MAC_DEVICE_CHEM  0x4B
#define MAC_DASTATUS1    0x71
#define MAC_QMAX         0x75

#define MAC_BLOCK_LEN 36

#define BQ_LOG(s, ...)                          \
    do {                                        \
        if ((s)->trace) {                       \
            fprintf(stderr, "BQ28Z610: ");      \
            fprintf(stderr, __VA_ARGS__);       \
        }                                       \
    } while (0)

/* 16-bit SBS value for a word register; the driver reads both bytes. */
static uint16_t bq28z610_word(Bq28z610State *s, uint8_t reg)
{
    switch (reg) {
    case SBS_CONTROL:     return s->control;
    case SBS_TEMPERATURE: return s->temperature_dk;
    case SBS_VOLTAGE:     return s->voltage_mv;
    case SBS_FLAGS:       return 0;   /* no alarms */
    case SBS_CURRENT:     return (uint16_t)s->current_ma;
    case SBS_AVG_CURRENT: return (uint16_t)s->current_ma;
    case SBS_REMAINING:   return s->remaining_mah;
    case SBS_FULL:        return s->full_mah;
    case SBS_CYCLES:      return s->cycles;
    case SBS_SOC:         return s->soc;
    case SBS_SOH:         return s->soh;
    case SBS_CHG_VOLTAGE: return s->chg_voltage_mv;
    case SBS_CHG_CURRENT: return s->chg_current_ma;
    case SBS_DESIGN_CAP:  return s->design_mah;
    case SBS_TTE:
        return s->current_ma > 0 ? (uint16_t)(s->remaining_mah * 60 /
                                              s->current_ma) : 0xffff;
    case SBS_TTF:         return 0;
    default:              return 0;
    }
}

static uint8_t bq28z610_checksum(const uint8_t *buf, unsigned len)
{
    unsigned i;
    uint8_t sum = 0;

    for (i = 0; i < len; i++) {
        sum += buf[i];
    }
    return (uint8_t)(0xFF - sum);
}

/* Build the MAC response block for an 8-bit command. */
static void bq28z610_mac_build(Bq28z610State *s, uint8_t cmd)
{
    uint8_t *b = s->mac_buf;
    unsigned data_len;

    memset(b, 0, MAC_BLOCK_LEN);
    b[0] = cmd;
    b[1] = 0;

    switch (cmd) {
    case MAC_DEVICE_NAME:
        memcpy(&b[2], "bq28z610", 8);
        data_len = 8;
        break;
    case MAC_MANU_NAME:
        memcpy(&b[2], "MITechnology", 11);
        data_len = 11;
        break;
    case MAC_DEVICE_CHEM:
        b[2] = 0x01;
        b[3] = 'L';    /* LWN cells */
        b[4] = 0x00;
        data_len = 3;
        break;
    case MAC_DASTATUS1:
        b[2] = s->cell_mv[0] & 0xff;
        b[3] = s->cell_mv[0] >> 8;
        b[4] = s->cell_mv[1] & 0xff;
        b[5] = s->cell_mv[1] >> 8;
        data_len = 4;
        break;
    case MAC_QMAX:
        b[2] = s->full_mah & 0xff;
        b[3] = s->full_mah >> 8;
        data_len = 2;
        break;
    default:
        BQ_LOG(s, "MAC 0x%02x not implemented\n", cmd);
        data_len = 0;
        break;
    }
    /* checksum covers the command echo plus the payload. */
    b[34] = bq28z610_checksum(b, 2 + data_len);
    b[35] = 2 + data_len + 2;
    s->mac_len = MAC_BLOCK_LEN;
    BQ_LOG(s, "MAC 0x%02x built, data_len=%u len=%u\n",
           cmd, data_len, b[35]);
}

static int bq28z610_event(I2CSlave *i2c, enum i2c_event event)
{
    Bq28z610State *s = BQ28Z610(i2c);

    switch (event) {
    case I2C_START_SEND:
        s->ptr_valid = false;
        break;
    case I2C_START_RECV:
        if (s->ptr >= SBS_ALT_MAC && s->ptr <= SBS_MAC_LEN) {
            unsigned off = s->ptr - SBS_ALT_MAC;

            s->rx[0] = off < s->mac_len ? s->mac_buf[off] : 0;
            s->rx[1] = off + 1 < s->mac_len ? s->mac_buf[off + 1] : 0;
            s->rx_len = 2;
        } else if (s->ptr <= SBS_DESIGN_CAP) {
            uint16_t word = bq28z610_word(s, s->ptr & ~1);

            s->rx[0] = (s->ptr & 1) ? (word >> 8) : (word & 0xff);
            s->rx[1] = (s->ptr & 1) ? 0 : (word >> 8);
            s->rx_len = 2;
        } else {
            /* Vendor registers (>=0x70) and unused addresses read as zero. */
            s->rx[0] = 0;
            s->rx[1] = 0;
            s->rx_len = 2;
        }
        s->rx_pos = 0;
        break;
    case I2C_FINISH:
    case I2C_NACK:
    default:
        break;
    }
    return 0;
}

static int bq28z610_send(I2CSlave *i2c, uint8_t data)
{
    Bq28z610State *s = BQ28Z610(i2c);

    if (!s->ptr_valid) {
        s->ptr = data;
        s->ptr_valid = true;
        return 0;
    }

    /* Data byte for the selected register. */
    if (s->ptr == SBS_ALT_MAC) {
        s->mac_cmd_lo = data;
    } else if (s->ptr == SBS_ALT_MAC + 1) {
        /* The driver writes 0 here; the command is complete. */
        bq28z610_mac_build(s, s->mac_cmd_lo);
    } else if (s->ptr == SBS_CONTROL) {
        s->control = (s->control & 0xff00) | data;
    } else if (s->ptr == SBS_CONTROL + 1) {
        s->control = (s->control & 0x00ff) | (data << 8);
    }
    BQ_LOG(s, "write %#04x = %#04x\n", s->ptr, data);
    return 0;
}

static uint8_t bq28z610_recv(I2CSlave *i2c)
{
    Bq28z610State *s = BQ28Z610(i2c);
    uint8_t v = 0;

    if (s->rx_pos < s->rx_len) {
        v = s->rx[s->rx_pos++];
    }
    return v;
}

static void bq28z610_reset(DeviceState *dev)
{
    Bq28z610State *s = BQ28Z610(dev);

    s->ptr = 0;
    s->ptr_valid = false;
    s->rx_len = s->rx_pos = 0;
    s->mac_len = 0;
    s->mac_cmd_lo = 0;
    s->control = 0;
    memset(s->mac_buf, 0, sizeof(s->mac_buf));
}

static void bq28z610_realize(DeviceState *dev, Error **errp)
{
    Bq28z610State *s = BQ28Z610(dev);

    bq28z610_reset(dev);
    s->trace = getenv("MTK_BQLOG") != NULL;
    s->cell_mv[0] = s->voltage_mv / 2;
    s->cell_mv[1] = s->voltage_mv - s->cell_mv[0];
    if (s->full_mah && !s->remaining_mah) {
        s->remaining_mah = (uint32_t)s->full_mah * s->soc / 100;
    }
    fprintf(stderr, "BQ28Z610: model at 0x55: %u mV, %u%%, %d mA, %u/%u mAh\n",
            s->voltage_mv, s->soc, s->current_ma, s->remaining_mah,
            s->full_mah);
}

static const Property bq28z610_props[] = {
    DEFINE_PROP_UINT16("voltage-mv", Bq28z610State, voltage_mv, 3850),
    DEFINE_PROP_INT32("current-ma", Bq28z610State, current_ma, 500),
    DEFINE_PROP_UINT16("temperature-dk", Bq28z610State, temperature_dk, 2981),
    DEFINE_PROP_UINT16("soc", Bq28z610State, soc, 55),
    DEFINE_PROP_UINT16("soh", Bq28z610State, soh, 95),
    DEFINE_PROP_UINT16("design-mah", Bq28z610State, design_mah, 5000),
    DEFINE_PROP_UINT16("full-mah", Bq28z610State, full_mah, 5000),
    DEFINE_PROP_UINT16("remaining-mah", Bq28z610State, remaining_mah, 2750),
    DEFINE_PROP_UINT16("cycles", Bq28z610State, cycles, 10),
    DEFINE_PROP_UINT16("chg-voltage-mv", Bq28z610State, chg_voltage_mv, 4400),
    DEFINE_PROP_UINT16("chg-current-ma", Bq28z610State, chg_current_ma, 3000),
};

static void bq28z610_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->event = bq28z610_event;
    k->send = bq28z610_send;
    k->recv = bq28z610_recv;
    dc->realize = bq28z610_realize;
    device_class_set_legacy_reset(dc, bq28z610_reset);
    device_class_set_props(dc, bq28z610_props);
    dc->desc = "TI bq28z610 battery fuel gauge (SBS + MAC)";
}

static const TypeInfo bq28z610_types[] = {
    {
        .name = TYPE_BQ28Z610,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(Bq28z610State),
        .class_init = bq28z610_class_init,
    },
};

DEFINE_TYPES(bq28z610_types)
