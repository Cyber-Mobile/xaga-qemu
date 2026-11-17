/*
 * MediaTek GCE / CMDQ sync events: interface shared with peripheral models.
 *
 * Display CMDQ packets end with a wait-for-event (WFE) on the display
 * frame-done events, and the display blocks raise those events from their
 * own frame timers: DSI0 raises disp_dsi0_sof0 / disp_dsi0_eof once per
 * video frame, the DISP mutex raises disp_mutex0_eof.  A GCE thread that
 * reaches a WFE blocks until the event bit is set (and, for wait-and-clear,
 * clears it), which is what gates the page-flip completion callback.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_MTK_GCE_H
#define HW_MISC_MTK_GCE_H

struct DeviceState;

/* GCE event ids (runtime device tree, dispsys_config "gce-events"). */
#define MTK_GCE_EVENT_DSI0_SOF      278u    /* disp_dsi0_sof0 */
#define MTK_GCE_EVENT_DSI0_EOF      313u    /* disp_dsi0_eof */
#define MTK_GCE_EVENT_MUTEX0_EOF    342u    /* disp_mutex0_eof */

/*
 * Set (raise) CMDQ event @event: wakes every thread blocked in a WFE on it.
 * Called by the peripheral frame timers; safe from a QEMUTimer callback.
 */
void mtk_gce_event_signal(struct DeviceState *dev, unsigned event);

/*
 * Declare that this model raises @event on its own.  A WFE that waits on an
 * event nobody generates keeps the legacy non-blocking behaviour, so a
 * missing event model can never hang a guest commit packet.
 */
void mtk_gce_event_mark_generated(struct DeviceState *dev, unsigned event);

#endif /* HW_MISC_MTK_GCE_H */
