/*
 * MediaTek SoC system timer (SYST) clock-event block
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_TIMER_MTK_SYSTIMER_H
#define HW_TIMER_MTK_SYSTIMER_H

#include "qom/object.h"

#define TYPE_MTK_SYSTIMER "mtk-systimer"
OBJECT_DECLARE_SIMPLE_TYPE(MtkSysTimerState, MTK_SYSTIMER)

#endif /* HW_TIMER_MTK_SYSTIMER_H */
