/*
 * MediaTek MT6895 AFE (audio front end) model — QEMU side.
 *
 * This is a first-stage skeleton: it exposes the AFE register window at
 * 0x1e100000 (DT: /mt6895-afe-pcm@1e100000, reg 0x2000, GIC SPI 416) with a
 * plain register file, an auto-detected memif ring buffer in guest DRAM, and a
 * QEMU audio backend voice so the host speaker can be driven.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_AUDIO_MTK_AFE_H
#define HW_AUDIO_MTK_AFE_H

#include "qom/object.h"

#define TYPE_MTK_AFE "mtk-afe"
OBJECT_DECLARE_SIMPLE_TYPE(MtkAfeState, MTK_AFE)

/* DT: mt6895-afe-pcm@1e100000 has reg = <0 0x1e100000 0 0x2000>. */
#define MTK_AFE_MMIO_SIZE       0x2000

#endif /* HW_AUDIO_MTK_AFE_H */
