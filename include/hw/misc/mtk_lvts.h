/*
 * MediaTek MT6895 LVTS thermal controller & TIA ADC device
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_MTK_LVTS_H
#define HW_MISC_MTK_LVTS_H

#include "hw/core/qdev.h"
#include "qom/object.h"

#define TYPE_MTK_LVTS "mtk-lvts"
OBJECT_DECLARE_SIMPLE_TYPE(MtkLvtsState, MTK_LVTS)

#define MTK_LVTS_D0_BASE    0x10315000ULL
#define MTK_LVTS_D1_BASE    0x10316000ULL
#define MTK_LVTS_WIN_SIZE   0x1000ULL

#define MTK_TIA_BASE        0x1c805000ULL
#define MTK_TIA_WIN_SIZE    0x1000ULL

#endif /* HW_MISC_MTK_LVTS_H */
