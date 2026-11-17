/*
 * MediaTek I2C controller (v2 register map) + KTZ8863A slave.
 *
 * Register layout taken from the offset table LK's driver indexes
 * (lk_a.img payload 0x8e75e, halfwords): DATA_PORT 0x00, SLAVE_ADDR 0x94,
 * INTR_MASK 0x08, INTR_STAT 0x0c, CONTROL 0x10, TRANSFER_LEN 0x14,
 * TRANSAC_LEN 0x18, DELAY_LEN 0x1c, TIMING 0x20, START 0x24, EXT_CONF 0x28,
 * LTIMING 0x2c, HS 0x30, IO_CONFIG 0x34, FIFO_ADDR_CLR 0x38,
 * TRANSFER_LEN_AUX 0x44, CLOCK_DIV 0x48, SOFTRESET 0x50, DEBUGSTAT 0xe4,
 * DEBUGCTRL 0xe8, FIFO_STAT 0xf4, FIFO_THRESH 0xf8, DCM_EN 0xf88.
 *
 * LK's _i2c_deal_result (payload 0x20ba8) programs SLAVE_ADDR = addr<<1|rd,
 * TRANSFER_LEN / TRANSFER_LEN_AUX / TRANSAC_LEN, pushes the TX bytes into
 * DATA_PORT (FIFO mode), writes START=1 and then polls INTR_STAT up to
 * 0xffff times: bit1|bit2 (ACKERR / HS_NACKERR) -> error, bit0
 * (TRANSAC_COMP) -> done.  INTR_STAT is write-1-to-clear.
 *
 * Transfers complete synchronously on the START write and are carried out
 * on a real QEMU I2CBus, so a missing slave produces ACKERR (as on silicon)
 * instead of a timeout.  DMA mode (CONTROL bit2) is not modelled yet; such a
 * transfer is reported as an ACK error and logged under LOG_UNIMP.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/i2c/mtk_i2c.h"

#define R_DATA_PORT        0x00
#define R_INTR_MASK        0x08
#define R_INTR_STAT        0x0c
#define R_CONTROL          0x10
#define R_TRANSFER_LEN     0x14
#define R_TRANSAC_LEN      0x18
#define R_START            0x24
#define R_FIFO_ADDR_CLR    0x38
#define R_TRANSFER_LEN_AUX 0x44
#define R_SOFTRESET        0x50
#define R_SLAVE_ADDR       0x94
#define R_FIFO_STAT        0xf4

#define INTR_TRANSAC_COMP  (1u << 0)
#define INTR_ACKERR        (1u << 1)
#define INTR_HS_NACKERR    (1u << 2)

#define CTL_RS             (1u << 1)
#define CTL_DMA_EN         (1u << 2)
#define CTL_DIR_CHANGE     (1u << 4)

static void mtk_i2c_update_irq(MtkI2CState *s)
{
    qemu_set_irq(s->irq, !!(s->regs[R_INTR_STAT / 4] & s->regs[R_INTR_MASK / 4]));
}

static void mtk_i2c_fifo_clear(MtkI2CState *s)
{
    s->tx_n = 0;
    s->rx_n = 0;
    s->rx_rd = 0;
}

/* One write phase: start(addr, W), push len bytes from the TX FIFO. */
static bool mtk_i2c_do_write(MtkI2CState *s, uint8_t addr7, unsigned len,
                             unsigned *txpos)
{
    unsigned i;

    if (i2c_start_send(s->bus, addr7)) {
        return false;
    }
    for (i = 0; i < len; i++) {
        uint8_t b = *txpos < s->tx_n ? s->tx_fifo[*txpos] : 0;

        (*txpos)++;
        if (i2c_send(s->bus, b)) {
            return false;
        }
    }
    return true;
}

static bool mtk_i2c_do_read(MtkI2CState *s, uint8_t addr7, unsigned len)
{
    unsigned i;

    if (i2c_start_recv(s->bus, addr7)) {
        return false;
    }
    for (i = 0; i < len; i++) {
        uint8_t b = i2c_recv(s->bus);

        if (s->rx_n < MTK_I2C_FIFO_SIZE) {
            s->rx_fifo[s->rx_n++] = b;
        }
    }
    return true;
}

static void mtk_i2c_start(MtkI2CState *s)
{
    uint32_t ctl = s->regs[R_CONTROL / 4];
    uint32_t sa = s->regs[R_SLAVE_ADDR / 4];
    uint8_t addr7 = (sa >> 1) & 0x7f;
    bool rd = sa & 1;
    unsigned len = s->regs[R_TRANSFER_LEN / 4] & 0xffff;
    unsigned aux = s->regs[R_TRANSFER_LEN_AUX / 4] & 0xffff;
    unsigned nx = s->regs[R_TRANSAC_LEN / 4] & 0xff;
    unsigned txpos = 0;
    unsigned t;
    bool ok = true;

    if (nx == 0) {
        nx = 1;
    }
    if (ctl & CTL_DMA_EN) {
        qemu_log_mask(LOG_UNIMP, "mtk-i2c%u: DMA mode not modelled\n", s->id);
        ok = false;
    } else if (ctl & CTL_DIR_CHANGE) {
        /* write_read: write len bytes, repeated start, read aux bytes */
        s->rx_n = s->rx_rd = 0;
        ok = mtk_i2c_do_write(s, addr7, len, &txpos) &&
             mtk_i2c_do_read(s, addr7, aux ? aux : 1);
        i2c_end_transfer(s->bus);
    } else if (rd) {
        s->rx_n = s->rx_rd = 0;
        for (t = 0; t < nx && ok; t++) {
            ok = mtk_i2c_do_read(s, addr7, len);
            if (!(ctl & CTL_RS) || t + 1 == nx) {
                i2c_end_transfer(s->bus);
            }
        }
    } else {
        for (t = 0; t < nx && ok; t++) {
            ok = mtk_i2c_do_write(s, addr7, len, &txpos);
            if (!(ctl & CTL_RS) || t + 1 == nx) {
                i2c_end_transfer(s->bus);
            }
        }
    }
    if (!ok) {
        i2c_end_transfer(s->bus);
    }
    s->tx_n = 0;
    s->regs[R_INTR_STAT / 4] |= INTR_TRANSAC_COMP | (ok ? 0 : INTR_ACKERR);
    s->regs[R_START / 4] &= ~1u;
    if (s->trace) {
        fprintf(stderr, "MTK-I2C%u: addr=%#x %s len=%u aux=%u nx=%u ctl=%#x "
                "-> %s\n", s->id, addr7,
                (ctl & CTL_DIR_CHANGE) ? "wr-rd" : (rd ? "rd" : "wr"),
                len, aux, nx, ctl, ok ? "ACK" : "NACK");
    }
    mtk_i2c_update_irq(s);
}

static uint64_t mtk_i2c_read(void *opaque, hwaddr off, unsigned size)
{
    MtkI2CState *s = opaque;
    uint32_t v;

    off &= ~3ULL;
    switch (off) {
    case R_DATA_PORT:
        v = s->rx_rd < s->rx_n ? s->rx_fifo[s->rx_rd++] : 0;
        break;
    case R_FIFO_STAT:
        /* [3:0] rd fifo count, [7:4] wr fifo count; bit... keep counts only */
        v = ((s->rx_n - s->rx_rd) & 0xf) | ((s->tx_n & 0xf) << 4);
        break;
    default:
        v = off < sizeof(s->regs) ? s->regs[off / 4] : 0;
        break;
    }
    return v;
}

static void mtk_i2c_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    MtkI2CState *s = opaque;

    off &= ~3ULL;
    switch (off) {
    case R_DATA_PORT:
        if (s->tx_n < MTK_I2C_FIFO_SIZE) {
            s->tx_fifo[s->tx_n++] = val;
        }
        return;
    case R_INTR_STAT:
        s->regs[R_INTR_STAT / 4] &= ~(uint32_t)val;
        mtk_i2c_update_irq(s);
        return;
    case R_FIFO_ADDR_CLR:
        if (val & 1) {
            mtk_i2c_fifo_clear(s);
        }
        return;
    case R_SOFTRESET:
        if (val & 1) {
            mtk_i2c_fifo_clear(s);
            s->regs[R_INTR_STAT / 4] = 0;
            mtk_i2c_update_irq(s);
        }
        return;
    case R_START:
        s->regs[R_START / 4] = val;
        if (val & 1) {
            mtk_i2c_start(s);
        }
        return;
    default:
        if (off < sizeof(s->regs)) {
            s->regs[off / 4] = val;
        }
        if (off == R_INTR_MASK) {
            mtk_i2c_update_irq(s);
        }
        return;
    }
}

static const MemoryRegionOps mtk_i2c_ops = {
    .read = mtk_i2c_read,
    .write = mtk_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

static void mtk_i2c_reset(DeviceState *dev)
{
    MtkI2CState *s = MTK_I2C(dev);

    memset(s->regs, 0, sizeof(s->regs));
    mtk_i2c_fifo_clear(s);
}

static void mtk_i2c_realize(DeviceState *dev, Error **errp)
{
    MtkI2CState *s = MTK_I2C(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &mtk_i2c_ops, s,
                          TYPE_MTK_I2C, MTK_I2C_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->bus = i2c_init_bus(dev, "i2c");
}

static const Property mtk_i2c_props[] = {
    DEFINE_PROP_UINT32("id", MtkI2CState, id, 0),
    DEFINE_PROP_BOOL("trace", MtkI2CState, trace, false),
};

static void mtk_i2c_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = mtk_i2c_realize;
    device_class_set_legacy_reset(dc, mtk_i2c_reset);
    device_class_set_props(dc, mtk_i2c_props);
    dc->desc = "MediaTek I2C controller (v2 register map)";
}

/*
 * KTZ8863A: LCD backlight + bias (AVDD/AVEE) controller, 7-bit address 0x11.
 * Modelled as a plain 8-bit-addressed register file with auto-increment
 * (first byte of a write = register pointer).  Only what the guest writes is
 * stored; reset values are left at 0 because the datasheet reset values /
 * revision register are not available to this project -- nothing is preset.
 */
#define TYPE_KTZ8863A_IMPL TYPE_KTZ8863A
OBJECT_DECLARE_SIMPLE_TYPE(Ktz8863aState, KTZ8863A_IMPL)

struct Ktz8863aState {
    I2CSlave parent_obj;
    uint8_t regs[256];
    uint8_t ptr;
    bool first;
    bool trace;
};

static int ktz8863a_event(I2CSlave *i2c, enum i2c_event ev)
{
    Ktz8863aState *s = KTZ8863A_IMPL(i2c);

    if (ev == I2C_START_SEND) {
        s->first = true;
    }
    return 0;
}

static int ktz8863a_send(I2CSlave *i2c, uint8_t data)
{
    Ktz8863aState *s = KTZ8863A_IMPL(i2c);

    if (s->first) {
        s->ptr = data;
        s->first = false;
    } else {
        if (s->trace) {
            fprintf(stderr, "KTZ8863A: reg[%#04x] = %#04x\n", s->ptr, data);
        }
        s->regs[s->ptr++] = data;
    }
    return 0;
}

static uint8_t ktz8863a_recv(I2CSlave *i2c)
{
    Ktz8863aState *s = KTZ8863A_IMPL(i2c);

    return s->regs[s->ptr++];
}

static void ktz8863a_reset(DeviceState *dev)
{
    Ktz8863aState *s = KTZ8863A_IMPL(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->ptr = 0;
    s->first = false;
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
    device_class_set_legacy_reset(dc, ktz8863a_reset);
    device_class_set_props(dc, ktz8863a_props);
    dc->desc = "KTZ8863A LCD backlight/bias controller";
}

static const TypeInfo mtk_i2c_types[] = {
    {
        .name = TYPE_MTK_I2C,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(MtkI2CState),
        .class_init = mtk_i2c_class_init,
    }, {
        .name = TYPE_KTZ8863A,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(Ktz8863aState),
        .class_init = ktz8863a_class_init,
    },
};

DEFINE_TYPES(mtk_i2c_types)
