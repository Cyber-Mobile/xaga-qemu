#!/usr/bin/env bash
# Configure and build qemu-system-aarch64 inside the fork.
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p build
cd build
../configure --target-list=aarch64-softmmu --disable-docs --disable-tools \
  --disable-guest-agent --disable-gtk --disable-sdl --disable-curses \
  --disable-opengl --disable-libssh --disable-capstone
ninja -j"$(nproc 2>/dev/null || echo 8)" qemu-system-aarch64
echo "built: build/qemu-system-aarch64"
