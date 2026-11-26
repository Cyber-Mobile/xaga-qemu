#!/usr/bin/env bash
# Boot the emulated xaga with vendor dumps placed in fw/ and data/.
#
# Default: zero-injection normal boot, 8 cores, deterministic timing.
# Benchmark parallel translation: scripts/run.sh --mttcg
# Recovery:  MTK_BOOT_MODE=2 MTK_TOUCH=1 MTK_USBDEV=1 scripts/run.sh
# Any MTK_* diagnostics are opt-in; none are required for the boot to advance.
set -euo pipefail
cd "$(dirname "$0")/.."
tcg_args=(-accel tcg,thread=single,tb-size=4096 -icount shift=4,sleep=off)
if [[ ${1:-} == --mttcg ]]; then
  tcg_args=(-accel tcg,thread=multi,tb-size=4096)
  shift
fi
exec ./build/qemu-system-aarch64 \
  -M mt6895,preloader=fw/LUA_1,gpt=fw/gpt_512.bin,data=data/lu2.img,entry-bytes=3072 \
  -m 6G -smp 8 -display none -monitor none \
  "${tcg_args[@]}" -serial file:OUT.log "$@"
