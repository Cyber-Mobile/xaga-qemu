/*
 * MediaTek MT6375 Sub-PMIC + Type-C Port Controller (TCPCI) + BC1.2 model.
 *
 * Connected on MT6895 i2c5@11280000 (7-bit primary address 0x34, IRQ line
 * wired to EINT pin 0x74 = instance 1, index 18).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_I2C_MT6375_H
#define HW_I2C_MT6375_H

#include "hw/i2c/i2c.h"
#include "qom/object.h"

#define TYPE_MT6375 "mt6375"
OBJECT_DECLARE_SIMPLE_TYPE(Mt6375State, MT6375)

DeviceState *mt6375_attach(I2CBus *bus, qemu_irq irq);

#endif /* HW_I2C_MT6375_H */
