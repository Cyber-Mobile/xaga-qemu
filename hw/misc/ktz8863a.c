/*
 * Kinetic Technologies KTZ8863A: high-efficiency 3-channel LED backlight
 * driver with dual LCD bias power (boost + LDO/inverting charge pump).
 *
 * The real chip is a pure I2C slave at 7-bit address 0x11; this model covers
 * its register interface only (the analog LED/bias behaviour is not modelled).
 *
 * Sources:
 *   - Kinetic Technologies, "KTZ8863A High Efficiency 3-CH LED Backlight
 *     Driver with Dual LCD Bias Power", Preliminary Datasheet, October 2020,
 *     Revision 04b, https://www.kinet-ic.com/uploads/web/KTZ8863A/KTZ8863A-04b.pdf
 *       p.24  I2C write/read cycles; 7-bit slave address 0b0010001 = 0x11;
 *             register pointer auto-increments after each acknowledged byte
 *       p.25  Table 7   I2C register map (0x01..0x15) with default values
 *       p.25  Table 8   REV: bits[7:2] die revision, bits[1:0] = 0b10
 *                       (Kinetic vendor ID); reset value 0x12
 *       p.26-33 Tables 9-28  per-register bit fields and reset values
 *   - Linux vendor driver drivers/gpu/drm/panel/leds-ktz8863a.c from the
 *     Xiaomi/LineageOS xaga kernel (5.10.269).  It uses the same addresses
 *     with its own names (BL_CFG1 -> BC1, BL_BRT_LSB/MSB -> BB_LSB/BB_MSB,
 *     LCD_BIAS_CFG1 -> BIAS_CONF1, LCD_BOOST_CFG -> BIAS_BOOST, OUTP_CFG ->
 *     BIAS_VPOS, OUTN_CFG -> BIAS_VNEG, FLAG -> FLAGS, BL_OPTION1/2 ->
 *     OPTION1/2, PWM2DIG_LSB/MSB -> PTD_LSB/PTD_MSB) and drives the panel
 *     backlight as an 11-bit code split over 0x04[2:0] and 0x05[7:0], with
 *     the enable in 0x08 bit[4].
 *
 * Register pointer semantics (datasheet p.24): the first data byte of a write
 * cycle is the register address; it increments after every byte the slave
 * acknowledges.  A read cycle returns data from the register addressed by the
 * preceding write cycle (repeated START, or a separate address-only write as
 * done by i2c_master_send()/i2c_master_recv() in the Linux driver).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/qdev-properties.h"
#include "hw/i2c/i2c.h"
#include "hw/misc/ktz8863a.h"

/* 8-bit register address space; only 0x01..0x15 are documented. */
#define KTZ8863A_NUM_REGS 256

/* Register addresses: datasheet Table 7 (p.25). */
enum {
    REG_REV           = 0x01,
    REG_BL_CFG1       = 0x02,
    REG_BL_CFG2       = 0x03,
    REG_BL_BRT_LSB    = 0x04,
    REG_BL_BRT_MSB    = 0x05,
    REG_BL_AUTOF_LOW  = 0x06,
    REG_BL_AUTOF_HIGH = 0x07,
    REG_BL_EN         = 0x08,
    REG_LCD_BIAS_CFG1 = 0x09,
    REG_LCD_BIAS_CFG2 = 0x0a,
    REG_LCD_BIAS_CFG3 = 0x0b,
    REG_LCD_BOOST_CFG = 0x0c,
    REG_OUTP_CFG      = 0x0d,
    REG_OUTN_CFG      = 0x0e,
    REG_FLAG          = 0x0f,
    REG_BL_OPTION1    = 0x10,
    REG_BL_OPTION2    = 0x11,
    REG_PWM2DIG_LSB   = 0x12,
    REG_PWM2DIG_MSB   = 0x13,
    REG_RAMP          = 0x14,
    REG_PWM_RAMP      = 0x15,
};

#define BL_EN_SW_RESET (1u << 7)
#define BL_EN_BL       (1u << 4)

/*
 * Power-on reset values, datasheet Table 7 (p.25); the per-bit meaning and
 * the individual reset values are documented in Tables 8-28 (p.25-33).
 */
static const uint8_t ktz8863a_reset_values[] = {
    [0x00] = 0x00,                     /* not documented */
    [REG_REV]           = 0x12,        /* die revision, vendor bits = 0b10 */
    [REG_BL_CFG1]       = 0x28,
    [REG_BL_CFG2]       = 0x8d,
    [REG_BL_BRT_LSB]    = 0x07,
    [REG_BL_BRT_MSB]    = 0xff,
    [REG_BL_AUTOF_LOW]  = 0x00,
    [REG_BL_AUTOF_HIGH] = 0x00,
    [REG_BL_EN]         = 0x00,
    [REG_LCD_BIAS_CFG1] = 0x18,
    [REG_LCD_BIAS_CFG2] = 0x11,        /* read-only */
    [REG_LCD_BIAS_CFG3] = 0x00,
    [REG_LCD_BOOST_CFG] = 0x28,
    [REG_OUTP_CFG]      = 0x1e,
    [REG_OUTN_CFG]      = 0x1c,
    [REG_FLAG]          = 0x00,        /* read-only status */
    [REG_BL_OPTION1]    = 0x06,
    [REG_BL_OPTION2]    = 0x35,
    [REG_PWM2DIG_LSB]   = 0x00,        /* read-only */
    [REG_PWM2DIG_MSB]   = 0x00,        /* read-only */
    [REG_RAMP]          = 0x44,
    [REG_PWM_RAMP]      = 0xf8,
};

struct Ktz8863aState {
    I2CSlave parent_obj;

    uint8_t regs[KTZ8863A_NUM_REGS];
    uint8_t ptr;            /* current register address pointer */
    bool expect_ptr;        /* next received byte is a register address */
    bool trace;             /* log every register access */
    bool bl_log;            /* log backlight updates (env MTK_BL_LOG=1) */
};

/* 11-bit backlight code: 8 MSBs in 0x05, 3 LSBs in 0x04[2:0] (Table 11/12). */
static unsigned ktz8863a_bl_code(const Ktz8863aState *s)
{
    return ((unsigned)s->regs[REG_BL_BRT_MSB] << 3) |
           (s->regs[REG_BL_BRT_LSB] & 0x7);
}

static void ktz8863a_log_backlight(Ktz8863aState *s, const char *reason)
{
    unsigned code = ktz8863a_bl_code(s);

    fprintf(stderr,
            "KTZ8863A: backlight %s level=%u/2047 (0x%03x) msb=0x%02x "
            "lsb=0x%02x bl_en=%u sinks=0x%x bias_en=%u\n",
            reason, code, code,
            s->regs[REG_BL_BRT_MSB], s->regs[REG_BL_BRT_LSB],
            (s->regs[REG_BL_EN] & BL_EN_BL) ? 1 : 0,
            s->regs[REG_BL_EN] & 0x7,
            (s->regs[REG_LCD_BIAS_CFG1] >> 7) & 1);
}

static bool ktz8863a_reg_is_read_only(unsigned reg)
{
    switch (reg) {
    case REG_REV:           /* die revision, Table 8 */
    case REG_LCD_BIAS_CFG2: /* Table 17 */
    case REG_FLAG:          /* Table 22 */
    case REG_PWM2DIG_LSB:   /* Table 25 */
    case REG_PWM2DIG_MSB:   /* Table 26 */
        return true;
    default:
        return false;
    }
}

static void ktz8863a_reset_regs(Ktz8863aState *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    memcpy(s->regs, ktz8863a_reset_values, sizeof(ktz8863a_reset_values));
    s->ptr = 0;
}

static int ktz8863a_event(I2CSlave *i2c, enum i2c_event event)
{
    Ktz8863aState *s = KTZ8863A_I2C(i2c);

    switch (event) {
    case I2C_START_SEND:
        /* First byte of a write cycle is the register address (datasheet p.24). */
        s->expect_ptr = true;
        break;
    case I2C_START_RECV:
        /* Repeated START for a read: keep the pointer from the write cycle. */
        s->expect_ptr = false;
        break;
    case I2C_FINISH:
    case I2C_NACK:
        s->expect_ptr = false;
        break;
    default:
        break;
    }
    return 0;
}

static int ktz8863a_send(I2CSlave *i2c, uint8_t data)
{
    Ktz8863aState *s = KTZ8863A_I2C(i2c);
    unsigned reg = s->ptr;

    if (s->expect_ptr) {
        s->ptr = data;
        s->expect_ptr = false;
        if (s->trace) {
            fprintf(stderr, "KTZ8863A: ptr <= %#04x\n", data);
        }
        return 0;
    }

    if (reg == REG_BL_EN && (data & BL_EN_SW_RESET)) {
        /*
         * Software reset: all registers return to their defaults and the
         * reset bit self-clears (datasheet Table 15 note).
         */
        if (s->trace || s->bl_log) {
            fprintf(stderr, "KTZ8863A: software reset\n");
        }
        ktz8863a_reset_regs(s);
        s->ptr = reg + 1;
        return 0;
    }

    if (!ktz8863a_reg_is_read_only(reg)) {
        s->regs[reg] = data;
    } else if (s->trace) {
        fprintf(stderr, "KTZ8863A: reg[%#04x] is read-only, write ignored\n",
                reg);
    }

    if (s->trace) {
        fprintf(stderr, "KTZ8863A: reg[%#04x] <= %#04x\n", reg, data);
    }
    if (s->bl_log && (reg == REG_BL_BRT_LSB || reg == REG_BL_BRT_MSB ||
                      reg == REG_BL_EN || reg == REG_LCD_BIAS_CFG1)) {
        ktz8863a_log_backlight(s, "update");
    }
    s->ptr = reg + 1;
    return 0;
}

static uint8_t ktz8863a_recv(I2CSlave *i2c)
{
    Ktz8863aState *s = KTZ8863A_I2C(i2c);
    unsigned reg = s->ptr;
    uint8_t val = s->regs[reg];

    if (s->trace) {
        fprintf(stderr, "KTZ8863A: reg[%#04x] => %#04x\n", reg, val);
    }
    if (reg == REG_FLAG) {
        /*
         * Latching protection flags are cleared by reading the register
         * (datasheet Table 22 note 2).  No fault is modelled, so only the
         * stored latched value has to be dropped here.
         */
        s->regs[REG_FLAG] = 0;
    }
    s->ptr = reg + 1;
    return val;
}

static void ktz8863a_reset(DeviceState *dev)
{
    Ktz8863aState *s = KTZ8863A_I2C(dev);

    ktz8863a_reset_regs(s);
    s->expect_ptr = true;
}

static void ktz8863a_realize(DeviceState *dev, Error **errp)
{
    Ktz8863aState *s = KTZ8863A_I2C(dev);
    const char *bl = getenv("MTK_BL_LOG");

    ktz8863a_reset_regs(s);
    s->expect_ptr = true;
    s->bl_log = bl && *bl && strcmp(bl, "0") != 0;
    if (s->bl_log) {
        fprintf(stderr, "KTZ8863A: model at 7-bit address 0x11, MTK_BL_LOG=1\n");
        ktz8863a_log_backlight(s, "reset");
    }
}

static const Property ktz8863a_props[] = {
    DEFINE_PROP_BOOL("trace", Ktz8863aState, trace, false),
};

static void ktz8863a_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->event = ktz8863a_event;
    k->send = ktz8863a_send;
    k->recv = ktz8863a_recv;
    dc->realize = ktz8863a_realize;
    device_class_set_legacy_reset(dc, ktz8863a_reset);
    device_class_set_props(dc, ktz8863a_props);
    dc->desc = "Kinetic KTZ8863A LED backlight + LCD bias controller";
}

static const TypeInfo ktz8863a_types[] = {
    {
        .name = TYPE_KTZ8863A_I2C,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(Ktz8863aState),
        .class_init = ktz8863a_class_init,
    },
};

DEFINE_TYPES(ktz8863a_types)
