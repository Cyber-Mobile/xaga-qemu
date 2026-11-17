/*
 * MediaTek M4U / IOMMU consumer interface (MT6895 model).
 *
 * The M4U translates DMA addresses (IOVA) issued by the multimedia/SMI
 * clients into guest physical addresses using ARMv7 short-descriptor page
 * tables with the MediaTek 34-bit IOVA / 35-bit PA extension.
 *
 * Translation is on by default (the hardware behaviour: every client is
 * attached to an IOMMU domain and programs IOVAs).  Environment overrides:
 *
 *   (unset) / MTK_IOMMU=1   real translation (default);
 *   MTK_IOMMU=0             no translation: clients program physical
 *                           addresses (the legacy bring-up setup where the
 *                           device tree iommus properties were NOPed out);
 *   MTK_DT_NOIOMMU=1        deprecated legacy hack, also disables translation.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_MTK_IOMMU_H
#define HW_MISC_MTK_IOMMU_H

#include <stdbool.h>
#include <stdint.h>

#include "exec/hwaddr.h"

/*
 * One instance per M4U hardware block (the bank-0 4 KiB register window is
 * the one the Linux driver programs; banks 1..4 are the protected/secure
 * banks owned by TF-A and are not translated here).
 */
enum {
    MTK_IOMMU_DEV_DISP = 0,     /* disp_iommu @ 0x1e802000 */
    MTK_IOMMU_DEV_MDP,          /* mdp_iommu  @ 0x1e810000 */
    MTK_IOMMU_DEV_APU0,         /* apu_iommu0 @ 0x19010000 */
    MTK_IOMMU_DEV_APU1,         /* apu_iommu1 @ 0x19015000 */
    MTK_IOMMU_DEV_NUM,
};

/* True when the machine is configured to model real M4U translation. */
bool mtk_iommu_enabled(void);

/*
 * Walk the bank-0 page table of @dev_id and translate @iova.
 *
 * @iova is 64-bit on purpose: the M4U is configured with ias = 34, so an IOVA
 * (e.g. a CMDQ command buffer dma address) can be above 4 GiB.  Returns false
 * (and latches a translation fault in the register file) when no valid
 * mapping exists.
 *
 * "Identity" fallback: buffers that live in the DRAM window but were not
 * mapped through the IOMMU (the LK framebuffer handed over to the kernel, or
 * a physical address programmed before a domain was attached) are returned
 * as-is instead of faulting.  This mirrors the hardware only for buffers
 * programmed by the bootloader; kernel IOVAs always resolve through the page
 * table first.
 */
bool mtk_iommu_translate(uint16_t dev_id, uint64_t iova, hwaddr *pa);

/*
 * Consumer helper: translate (with the identity fallback above) unless
 * MTK_IOMMU=0, in which case @dma is a physical address.  Every model that reads
 * guest memory on behalf of an M4U client uses this so the rule is defined in
 * one place.
 */
bool mtk_iommu_translate_or_identity(uint16_t dev_id, uint64_t dma,
                                     hwaddr *pa);

/*
 * Diagnostic: log the page-table entries and decoded physical address for
 * @iova on the bank-0 instance of @dev_id.  Called by consumer models when a
 * translated read fails, so the log shows whether the translation produced a
 * valid physical address that is simply not backed (model/consumer bug) or
 * whether the page-table walk itself is suspect.
 */
void mtk_iommu_dump_walk(uint16_t dev_id, uint64_t iova);

#endif /* HW_MISC_MTK_IOMMU_H */
