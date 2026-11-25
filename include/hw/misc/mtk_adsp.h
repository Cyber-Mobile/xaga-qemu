/*
 * MediaTek MT6895 ADSP subsystem and mailbox device
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_MTK_ADSP_H
#define HW_MISC_MTK_ADSP_H

#include "exec/hwaddr.h"
#include "hw/core/qdev.h"
#include "qom/object.h"

#define TYPE_MTK_ADSP "mtk-adsp"
OBJECT_DECLARE_SIMPLE_TYPE(MtkAdspState, MTK_ADSP)

#define MTK_ADSP_DTCM_SIZE 0x8000

uint64_t mtk_adsp_dtcm_read(DeviceState *dev, unsigned core, hwaddr off,
                            unsigned size);
void mtk_adsp_dtcm_write(DeviceState *dev, unsigned core, hwaddr off,
                         uint64_t val, unsigned size);

#endif /* HW_MISC_MTK_ADSP_H */
