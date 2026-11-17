/*
 * MediaTek SPMI PMIC arbiter (PMIF) host controller
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_MTK_SPMI_PMIF_H
#define HW_MISC_MTK_SPMI_PMIF_H

#include "hw/core/qdev.h"
#include "qom/object.h"

#define TYPE_MTK_SPMI_PMIF "mtk-spmi-pmif"
OBJECT_DECLARE_SIMPLE_TYPE(MtkSpmiPmifState, MTK_SPMI_PMIF)

/*
 * Write one PMIC register directly.  Used by the machine to record reset
 * causes the way the real PMIC latches them (e.g. the WDT status register),
 * which the boot chain reads to tell a cold boot from a watchdog reboot.
 */
void mtk_spmi_pmif_set_pmic_reg(DeviceState *dev, unsigned sid,
                                unsigned reg, uint8_t val);

/*
 * Emulate one MT6363 power key edge (the PMIC PWRKEY / PWRKEY_R interrupt).
 * The keypad model calls this for the phone's power key, which on this board is
 * wired to the PMIC's key block rather than to the keypad matrix.
 */
void mtk_spmi_pmif_power_key(DeviceState *dev, bool pressed);

#endif /* HW_MISC_MTK_SPMI_PMIF_H */
