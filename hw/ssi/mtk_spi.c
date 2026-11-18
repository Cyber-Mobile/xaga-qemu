/*
 * MediaTek MT6983 SPI controller (spi-mt65xx) master model.
 *
 * Register map and handshake are taken from the vendor driver in the tree
 * (firmware/ref/xaga-kernel/drivers/spi/spi-mt65xx.c):
 *   CFG0 0x00, CFG1 0x04 (bits[25:16] packet length-1), TX_SRC 0x08,
 *   RX_DST 0x0c, TX_DATA 0x10, RX_DATA 0x14, CMD 0x18 (ACT bit0,
 *   RESUME bit1, FINISH_IE bit16, PAUSE_IE bit17), STATUS0 0x1c (bit1 =
 *   pause status), STATUS1 0x20 (polled completion flag for the no-IRQ
 *   path), PAD_SEL 0x24, CFG2 0x28, CFG3 0x40.
 *
 * Transfer flow implemented here:
 *  - the driver programs CFG1 packet length, writes the TX FIFO words, sets
 *    FINISH_IE/PAUSE_IE and then ACT (or RESUME) via mtk_spi_enable_transfer;
 *  - the model clocks exactly (packet_length) bytes over the SSI bus, then
 *    completes after the real bus time, clears ACT/RESUME, latches STATUS0
 *    (no pause) and STATUS1 (=1, packet complete) and raises the IRQ;
 *  - the IRQ handler reads STATUS0 (read-clear), drains ceil(len/4) RX words
 *    from RX_DATA and finalises, or (multi-packet) programs the next packet
 *    and issues ACT again;
 *  - the no-IRQ path polls STATUS1 until it is non-zero and then drains RX.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/ssi/mtk_spi.h"
#include "exec/memattrs.h"
#include "system/address-spaces.h"

#define SPI_TRACE(s, ...)                                   \
    do {                                                    \
        if ((s)->trace) {                                   \
            fprintf(stderr, "MTK-SPI: ");                   \
            fprintf(stderr, __VA_ARGS__);                   \
        }                                                   \
    } while (0)

static void mtk_spi_update_irq(MtkSpiState *s)
{
    qemu_set_irq(s->irq, s->int_status != 0);
}

static void mtk_spi_xfer_done(void *opaque)
{
    MtkSpiState *s = opaque;

    s->busy = false;
    s->regs[MTK_SPI_REG_CMD / 4] &= ~(MTK_SPI_CMD_ACT | MTK_SPI_CMD_RESUME);
    s->regs[MTK_SPI_REG_STATUS0 / 4] = 0;      /* no pause: state = IDLE */
    s->regs[MTK_SPI_REG_STATUS1 / 4] = 1;      /* packet complete */
    s->int_status = 1;
    mtk_spi_update_irq(s);
    SPI_TRACE(s, "done bytes=%u rx_words=%u\n", s->packet_len, s->rx_count);
}

/* Latch the programmed packet length from CFG1[25:16] (length - 1). */
static uint32_t mtk_spi_packet_len(MtkSpiState *s)
{
    uint32_t len = ((s->regs[MTK_SPI_REG_CFG1 / 4] >> 16) & 0xffff) + 1;

    return len ? len : 1;
}

static void mtk_spi_start_xfer(MtkSpiState *s, bool resume)
{
    uint32_t nbytes = mtk_spi_packet_len(s);
    uint32_t i;
    int64_t ns;

    if (s->busy) {
        return;
    }
    if (nbytes > MTK_SPI_FIFO_WORDS * 4) {
        nbytes = MTK_SPI_FIFO_WORDS * 4;
    }
    s->packet_len = nbytes;
    {
        uint32_t cmd = s->regs[MTK_SPI_REG_CMD / 4];
        bool tx_dma = (cmd & BIT(11)) != 0;
        bool rx_dma = (cmd & BIT(10)) != 0;
        uint64_t tx_addr = s->regs[MTK_SPI_REG_TX_SRC / 4];
        uint64_t rx_addr = s->regs[MTK_SPI_REG_RX_DST / 4];
        uint8_t *txbuf = g_malloc0(nbytes);
        uint8_t *rxbuf = g_malloc0(nbytes);

        if (tx_dma) {
            address_space_read(&address_space_memory, tx_addr,
                               MEMTXATTRS_UNSPECIFIED, txbuf, nbytes);
        }
        qemu_set_irq(s->cs, 1);
        for (i = 0; i < nbytes; i++) {
            uint32_t tx;

            if (tx_dma) {
                tx = txbuf[i];
            } else {
                tx = (i / 4 < s->tx_count) ?
                    (s->tx_fifo[i / 4] >> ((i % 4) * 8)) & 0xff : 0;
            }
            rxbuf[i] = ssi_transfer(s->ssi, tx) & 0xff;
            s->rx_fifo[i / 4] =
                (s->rx_fifo[i / 4] & ~(0xffu << ((i % 4) * 8))) |
                ((uint32_t)rxbuf[i] << ((i % 4) * 8));
        }
        qemu_set_irq(s->cs, 0);
        if (rx_dma) {
            address_space_write(&address_space_memory, rx_addr,
                                MEMTXATTRS_UNSPECIFIED, rxbuf, nbytes);
        }
        g_free(txbuf);
        g_free(rxbuf);
    }
    s->rx_count = (nbytes + 3) / 4;
    s->rx_pos = 0;

    ns = (int64_t)nbytes * 8 * NANOSECONDS_PER_SECOND / s->clock_hz;
    if (ns < 500) {
        ns = 500;
    }
    s->busy = true;
    SPI_TRACE(s, "%s bytes=%u ns=%" PRId64 "\n",
              resume ? "resume" : "start", nbytes, ns);
    timer_mod(s->xfer_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ns);
}

static uint64_t mtk_spi_read(void *opaque, hwaddr off, unsigned size)
{
    MtkSpiState *s = opaque;
    uint64_t val;

    off &= ~3ULL;
    switch (off) {
    case MTK_SPI_REG_RXDATA:
        val = s->rx_pos < s->rx_count ? s->rx_fifo[s->rx_pos++] : 0;
        break;
    case MTK_SPI_REG_TXDATA:
        val = 0;
        break;
    case MTK_SPI_REG_STATUS0:
        /* Read-clear: the driver's IRQ handler starts by reading this. */
        val = s->regs[MTK_SPI_REG_STATUS0 / 4];
        s->int_status = 0;
        mtk_spi_update_irq(s);
        break;
    case MTK_SPI_REG_STATUS1:
        /*
         * Completion flag for the no-IRQ path: zero while the transfer is
         * running, non-zero once it has completed.
         */
        val = s->busy ? 0 : 1;
        break;
    case MTK_SPI_REG_CMD:
        val = s->regs[MTK_SPI_REG_CMD / 4];
        break;
    default:
        val = off < MTK_SPI_MMIO_SIZE ? s->regs[off / 4] : 0;
        break;
    }
    SPI_TRACE(s, "R %#03" HWADDR_PRIx " = %#" PRIx64 "\n", off, val);
    return val;
}

static void mtk_spi_write(void *opaque, hwaddr off, uint64_t value,
                          unsigned size)
{
    MtkSpiState *s = opaque;
    uint32_t old;

    off &= ~3ULL;
    SPI_TRACE(s, "W %#03" HWADDR_PRIx " = %#" PRIx64 "\n", off, value);
    switch (off) {
    case MTK_SPI_REG_TXDATA:
        if (s->tx_count < MTK_SPI_FIFO_WORDS) {
            s->tx_fifo[s->tx_count++] = (uint32_t)value;
        }
        break;
    case MTK_SPI_REG_CMD:
        old = s->regs[MTK_SPI_REG_CMD / 4];
        s->regs[MTK_SPI_REG_CMD / 4] = (uint32_t)value;
        /* ACT is edge-latched while the controller is idle (driver rule). */
        if (((uint32_t)value & MTK_SPI_CMD_ACT) && !s->busy &&
            !(old & MTK_SPI_CMD_ACT)) {
            s->tx_count_at_start = s->tx_count;
            mtk_spi_start_xfer(s, false);
            s->tx_count = 0;
        } else if (((uint32_t)value & MTK_SPI_CMD_RESUME) && !s->busy) {
            mtk_spi_start_xfer(s, true);
            s->tx_count = 0;
        }
        break;
    case MTK_SPI_REG_STATUS0:
        s->regs[MTK_SPI_REG_STATUS0 / 4] = (uint32_t)value;
        break;
    default:
        if (off < MTK_SPI_MMIO_SIZE) {
            s->regs[off / 4] = (uint32_t)value;
        }
        break;
    }
}

static const MemoryRegionOps mtk_spi_ops = {
    .read = mtk_spi_read,
    .write = mtk_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static void mtk_spi_reset_hold(Object *obj, ResetType type)
{
    MtkSpiState *s = MTK_SPI(obj);

    timer_del(s->xfer_timer);
    s->busy = false;
    s->tx_count = 0;
    s->tx_count_at_start = 0;
    s->rx_count = 0;
    s->rx_pos = 0;
    s->packet_len = 0;
    s->int_status = 0;
    memset(s->regs, 0, sizeof(s->regs));
    qemu_set_irq(s->irq, 0);
    qemu_set_irq(s->cs, 0);
}

static void mtk_spi_init(Object *obj)
{
    MtkSpiState *s = MTK_SPI(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->cs);
    memory_region_init_io(&s->iomem, obj, &mtk_spi_ops, s,
                          TYPE_MTK_SPI, MTK_SPI_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    s->ssi = ssi_create_bus(DEVICE(obj), "spi");
    s->trace = getenv("MTK_SPI_TRACE") != NULL;
}

static void mtk_spi_realize(DeviceState *dev, Error **errp)
{
    MtkSpiState *s = MTK_SPI(dev);

    s->xfer_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, mtk_spi_xfer_done, s);
    s->tx_fifo = g_new0(uint32_t, MTK_SPI_FIFO_WORDS);
    s->rx_fifo = g_new0(uint32_t, MTK_SPI_FIFO_WORDS);
}

static void mtk_spi_finalize(Object *obj)
{
    MtkSpiState *s = MTK_SPI(obj);

    timer_free(s->xfer_timer);
    g_free(s->tx_fifo);
    g_free(s->rx_fifo);
}

static const Property mtk_spi_props[] = {
    DEFINE_PROP_UINT32("clock-hz", MtkSpiState, clock_hz, 13000000),
};

static void mtk_spi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = mtk_spi_realize;
    device_class_set_props(dc, mtk_spi_props);
    dc->desc = "MediaTek MT6983 SPI controller";
    rc->phases.hold = mtk_spi_reset_hold;
}

static const TypeInfo mtk_spi_info = {
    .name = TYPE_MTK_SPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MtkSpiState),
    .instance_init = mtk_spi_init,
    .instance_finalize = mtk_spi_finalize,
    .class_init = mtk_spi_class_init,
};

static void mtk_spi_register_types(void)
{
    type_register_static(&mtk_spi_info);
}

type_init(mtk_spi_register_types)
