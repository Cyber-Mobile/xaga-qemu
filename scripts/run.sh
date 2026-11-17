#!/usr/bin/env bash
# Boot the emulated xaga with vendor dumps placed in fw/ and data/.
set -euo pipefail
cd "$(dirname "$0")/.."
export MTK_MEM=8G MTK_ICOUNT=off MTK_LU2_SNAP=1
export MTK_NO_NAMETBL=1 MTK_KINFO_FLAG0=1 MTK_LU2_RO=1 MTK_WDT_STATUS_VAL=1
export MTK_NO_ATFLOG=1 MTK_NO_ATFARG=1 MTK_NO_ATFARG2=1 MTK_GZ_SKIP=1 MTK_NO_GTAPLANT=1
exec ./build/qemu-system-aarch64 \
  -M mt6895,preloader=fw/LUA_1,gpt=fw/gpt_512.bin,data=data/lu2.img,entry-bytes=3072 \
  -m 8G -smp 1 -display none -monitor none -accel tcg,tb-size=256 \
  -serial file:OUT.log
