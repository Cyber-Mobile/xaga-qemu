/*
 * MediaTek MT6983 SPI controller (spi-mt65xx), as used on the MT6895 xaga.
 *
 * Register protocol decoded from the vendor driver module spi-mt65xx.ko:
 *   TX FIFO port 0x10, RX FIFO port 0x14, command register 0x18
 *   (bit0 = FIFO start, bit1 = DMA start, bits 10/11 = done status 0xc00,
 *   bits 16/17 = length mode 0x30000), interrupt status 0x1c (bit0 FIFO
 *   done, bit1 DMA done), CFG0..3 at 0x00/0x04/0x08/0x0c, pad select 0x2c.
 *
 * Transfers are byte-serial through a standard QEMU SSI bus: the controller
 * clocks out the TX FIFO words and clocks in the peripheral's responses, then
 * completes the transfer after the real bus time (bytes * 8 / bus clock) and
 * raises the completion interrupt; the start bits are edge triggered and are
 * cleared by the controller when the transfer finishes, exactly as the
 * driver expects.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SSI_MTK_SPI_H
#define HW_SSI_MTK_SPI_H

#include "hw/core/sysbus.h"
#include "hw/ssi/ssi.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_MTK_SPI "mtk-spi"
OBJECT_DECLARE_SIMPLE_TYPE(MtkSpiState, MTK_SPI)

#define MTK_SPI_MMIO_SIZE   0x100
#define MTK_SPI_FIFO_WORDS  16384   /* > 63 KiB firmware chunk / 4 */

#define MTK_SPI_REG_CFG0    0x00
#define MTK_SPI_REG_CFG1    0x04
#define MTK_SPI_REG_TX_SRC  0x08
#define MTK_SPI_REG_RX_DST  0x0c
#define MTK_SPI_REG_TXDATA  0x10
#define MTK_SPI_REG_RXDATA  0x14
#define MTK_SPI_REG_CMD     0x18
#define MTK_SPI_REG_STATUS0 0x1c
#define MTK_SPI_REG_STATUS1 0x20
#define MTK_SPI_REG_PADSEL  0x24
#define MTK_SPI_REG_CFG2    0x28
#define MTK_SPI_REG_TX_SRC64 0x2c
#define MTK_SPI_REG_RX_DST64 0x30
#define MTK_SPI_REG_CFG3    0x40

/* Command register bits (vendor driver spi-mt65xx.c). */
#define MTK_SPI_CMD_ACT       0x00000001u
#define MTK_SPI_CMD_RESUME    0x00000002u
#define MTK_SPI_CMD_FINISH_IE 0x00010000u
/* STATUS0 (0x1c) pause-interrupt status. */
#define MTK_SPI_STATUS0_PAUSE 0x00000002u

struct MtkSpiState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq cs;          /* chip select to the peripheral (active high) */

    QEMUTimer *xfer_timer;
    SSIBus *ssi;

    uint32_t regs[MTK_SPI_MMIO_SIZE / 4];
    uint32_t *tx_fifo;
    uint32_t *rx_fifo;
    unsigned tx_count;
    unsigned rx_count;
    unsigned rx_pos;

    bool busy;
    bool trace;
    uint32_t packet_len;
    unsigned tx_count_at_start;
    uint32_t int_status;
    uint32_t clock_hz;
};

#endif /* HW_SSI_MTK_SPI_H */
