/*
 * MediaTek I2C controller (v2 register map, MT6983/MT6895 class) and the
 * KTZ8863A LCD backlight / bias I2C slave.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_I2C_MTK_I2C_H
#define HW_I2C_MTK_I2C_H

#include "hw/core/sysbus.h"
#include "hw/i2c/i2c.h"
#include "qom/object.h"

#define TYPE_MTK_I2C "mtk-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(MtkI2CState, MTK_I2C)

#define TYPE_KTZ8863A "ktz8863a"

#define MTK_I2C_FIFO_SIZE 16
#define MTK_I2C_MMIO_SIZE 0x1000
#define MTK_I2C_DMA_SIZE  0x80

struct MtkI2CState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion dma_iomem;
    I2CBus *bus;
    qemu_irq irq;

    uint32_t regs[MTK_I2C_MMIO_SIZE / 4];
    uint32_t dma_regs[MTK_I2C_DMA_SIZE / 4];
    uint8_t tx_fifo[MTK_I2C_FIFO_SIZE];
    uint8_t tx_n;
    uint8_t rx_fifo[MTK_I2C_FIFO_SIZE];
    uint8_t rx_n;
    uint8_t rx_rd;
    uint32_t id;      /* bus number, only for trace output */
    bool trace;
};

#endif
