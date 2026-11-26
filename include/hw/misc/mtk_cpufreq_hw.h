/*
 * MediaTek MT6895 cpufreq-hw (CSRAM DVFS performance controller) device
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_MTK_CPUFREQ_HW_H
#define HW_MISC_MTK_CPUFREQ_HW_H

#include "hw/core/qdev.h"
#include "qom/object.h"

#define TYPE_MTK_CPUFREQ_HW "mtk-cpufreq-hw"
OBJECT_DECLARE_SIMPLE_TYPE(MtkCpufreqHwState, MTK_CPUFREQ_HW)

#define MTK_CPUFREQ_HW_BASE     0x0011bc10ULL
#define MTK_CPUFREQ_HW_DOMAINS  3
#define MTK_CPUFREQ_HW_DOM_SIZE 0x120ULL
#define MTK_CPUFREQ_HW_SIZE     (MTK_CPUFREQ_HW_DOMAINS * MTK_CPUFREQ_HW_DOM_SIZE)

#define MTK_CPUFREQ_EAS_LIMIT_BASE 0x0011ce08ULL
#define MTK_CPUFREQ_EAS_LIMIT_SIZE 0x10

#endif /* HW_MISC_MTK_CPUFREQ_HW_H */
