/*
 * Kinetic Technologies KTZ8863A: high-efficiency 3-channel LED backlight
 * driver with dual LCD bias power, modelled as an I2C slave.
 *
 * The chip has no CPU-visible interface besides I2C, so the device is a plain
 * TYPE_I2C_SLAVE; see hw/misc/ktz8863a.c for the register map and sources.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_KTZ8863A_H
#define HW_MISC_KTZ8863A_H

#include "hw/i2c/i2c.h"
#include "qom/object.h"

/*
 * Note the "-i2c" suffix: an older inline stub of the same chip is still
 * registered from hw/i2c/mtk_i2c.c as "ktz8863a", and two TypeInfos cannot
 * share a name.  The stub is dead once the machine instantiates this model
 * and can be deleted together with its TypeInfo.
 */
#define TYPE_KTZ8863A_I2C "ktz8863a-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(Ktz8863aState, KTZ8863A_I2C)

#endif /* HW_MISC_KTZ8863A_H */
