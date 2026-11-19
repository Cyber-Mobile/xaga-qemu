#!/usr/bin/env bash
# Boot the emulated xaga with vendor dumps placed in fw/ and data/.
#
# This is the recovery path that is exercised today. MTK_GZ_SKIP is the one named
# bypass still in use: it redirects ATF's BL32 hand-off to LK while the GenieZone
# stage is being brought up. Drop it (together with MTK_NO_GTAPLANT / MTK_NO_XHCI)
# to run the zero-injection ladder, which currently stops inside GZ.
set -euo pipefail
cd "$(dirname "$0")/.."
export MTK_MEM=6G MTK_LU2_SNAP=1
export MTK_KINFO_FLAG0=1 MTK_WDT_STATUS_VAL=1
export MTK_BOOT_MODE=2 MTK_PWR_STATE=1
export MTK_NO_ATFLOG=1 MTK_NO_GTAPLANT=1 MTK_NO_XHCI=1 MTK_GZ_SKIP=1
exec ./build/qemu-system-aarch64 \
  -M mt6895,preloader=fw/LUA_1,gpt=fw/gpt_512.bin,data=data/lu2.img,entry-bytes=3072 \
  -m 6G -smp 1 -display none -monitor none \
  -icount shift=6,sleep=off -accel tcg,tb-size=256 \
  -serial file:OUT.log
