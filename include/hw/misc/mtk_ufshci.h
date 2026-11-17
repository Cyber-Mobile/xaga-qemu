/*
 * MediaTek UFS host controller
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_MTK_UFSHCI_H
#define HW_MISC_MTK_UFSHCI_H

#include "qom/object.h"

/* Set once BL31 hands over to BL33 (LK). */
extern bool mtk_ufs_lk_stage;

#define TYPE_MTK_UFSHCI "mtk-ufshci"
OBJECT_DECLARE_SIMPLE_TYPE(MtkUfshciState, MTK_UFSHCI)

#endif /* HW_MISC_MTK_UFSHCI_H */
