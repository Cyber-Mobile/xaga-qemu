/*
 * MediaTek MT6895 Arm Mali-G610 control register block
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_MTK_MALI_GPU_H
#define HW_MISC_MTK_MALI_GPU_H

#include "hw/core/qdev.h"
#include "qom/object.h"

#define TYPE_MTK_MALI_GPU "mtk-mali-gpu"
OBJECT_DECLARE_SIMPLE_TYPE(MtkMaliGpuState, MTK_MALI_GPU)

#define MTK_MALI_GPU_BASE   0x13000000ULL
#define MTK_MALI_GPU_SIZE   0x00480000ULL

#define MTK_MALI_GPU_SPI    512
#define MTK_MALI_MMU_SPI    513
#define MTK_MALI_JOB_SPI    514

#endif /* HW_MISC_MTK_MALI_GPU_H */
