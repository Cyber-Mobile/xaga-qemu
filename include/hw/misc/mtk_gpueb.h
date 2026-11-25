/*
 * MediaTek MT6895 GPUEB (GPU Embedded Controller) boot & mailbox device
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_MTK_GPUEB_H
#define HW_MISC_MTK_GPUEB_H

#include "hw/core/qdev.h"
#include "qom/object.h"

#define TYPE_MTK_GPUEB "mtk-gpueb"
OBJECT_DECLARE_SIMPLE_TYPE(MtkGpuebState, MTK_GPUEB)

#define MTK_GPUEB_BOOT_BASE     0x13c5fd24ULL
#define MTK_GPUEB_BOOT_SIZE     0x1000ULL
#define MTK_GPUEB_MBOX0_BASE    0x13c5fd80ULL
#define MTK_GPUEB_MBOX0_SIZE    0x280ULL
#define MTK_GPUEB_MBOX0_SLOTS   160
#define MTK_GPUEB_IRQ_BASE      0x13c62000ULL
#define MTK_GPUEB_IRQ_SIZE      0x100ULL
#define MTK_GPUEB_SPI           516

#endif /* HW_MISC_MTK_GPUEB_H */
