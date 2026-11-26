/*
 * MediaTek MT6895 DPMAIF (Data Plane Modem AP Interface v3) device
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_MTK_DPMAIF_H
#define HW_MISC_MTK_DPMAIF_H

#include "hw/core/qdev.h"
#include "qom/object.h"

#define TYPE_MTK_DPMAIF "mtk-dpmaif"
OBJECT_DECLARE_SIMPLE_TYPE(MtkDpmaifState, MTK_DPMAIF)

#define MTK_DPMAIF_AO_BASE  0x10014000ULL
#define MTK_DPMAIF_PD_BASE  0x1022d000ULL
#define MTK_DPMAIF_WIN_SIZE 0x1000ULL

#endif /* HW_MISC_MTK_DPMAIF_H */
