/*
 * MediaTek syscon / register-block stub
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_MTK_SYSCON_H
#define HW_MISC_MTK_SYSCON_H

#define TYPE_MTK_SYSCON "mtk-syscon"

/*
 * Last value software wrote to the VLP clock-sys ULPOSC_CON0 register
 * (syscon window "vlp_cksys_clk", offset 0x200).  The topckgen frequency
 * meter needs it to report the ULPOSC frequency the preloader is calibrating;
 * the register lives in a different device, so it is exported like the other
 * cross-block latches in this model.
 */
extern uint32_t mtk_vlp_ulposc_con0;

#endif /* HW_MISC_MTK_SYSCON_H */
