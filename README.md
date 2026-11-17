# xaga emulator (MT6895)

**A full-system, scriptable simulator for the Xiaomi xaga (Redmi Note 11T Pro / POCO X4 GT, MediaTek MT6895).**

Goal: boot the *real* device firmware stack — preloader → BL2_EXT → ATF → TEE → LK → Linux → Android —
and faithfully emulate the hardware around it, so that recovery, fastboot, adb and the Android UI run
against the emulated phone instead of a model of it.

This project is implemented as a QEMU machine model (`-M mt6895`) on top of
[QEMU](https://www.qemu.org/) (fork at branch `xaga`, based on `v11.1.1`). **QEMU is the substrate;
the emulator is the project.**

![Recovery](docs/xaga/images/recovery-ui.png)

## What already works

| Area | State |
|---|---|
| Preloader → BL2_EXT → ATF → TEE → LK → Linux 5.10 boot chain | ✅ |
| Vendor Android userspace (boot to UI) | ✅ |
| Recovery mode (`MTK_BOOT_MODE=2`) + Xiaomi HyperOS Recovery UI (full screen, no blit artifacts) | ✅ |
| SMP: up to 8 cores online (lazy EL3 plant unlocks TEE/SiP SMC on secondary CPUs) | ✅ 4-core TEE path; A78 TEE path WIP |
| Display: OVL composition, dual-pipe half swap, MIPI DSI command queue, GCE/CMDQ execution, SimpleFB fallback | ✅ |
| Storage: UFS host controller (UIC/UTP/UCD), 4 KiB LUs, GPT, RPMB, block-backend (raw/snapshot/CoW) | ✅ |
| USB host: xHCI with keyboard / network / audio enumeration | ✅ |
| USB device mode: mtu3-style UDC + QMU, bridged to a host fastboot client (`getvar`, `download`, `flash`) | ✅ |
| KPD, RTC, PMIC/SPMI, regulators, clocks | ✅ partial (strict vendor power/clock checks WIP) |
| LineageOS: flash `boot`/`vendor_boot`/`dtbo`/`vbmeta`, boot to **recovery** (userspace + `adbd` started) | ✅ via `MTK_PWR_STATE=1` real power/clock modeling; default path unchanged |
| Audio: AFE model + 440 Hz self-test | 🧪 model ready |

## Roadmap

- [x] Boot chain, display, UFS, USB, recovery UI
- [x] LK fastboot over emulated USB device mode (host-side client included)
- [x] LineageOS kernel boot to recovery (`MTK_PWR_STATE=1` power/clock/bus-protection modeling)
- [ ] Full LineageOS/Android boot and `adb sideload`
- [ ] Kernel USB gadget: `fastbootd`, `adb`, `adb sideload` in recovery
- [ ] Audio path end-to-end (AFE + MT6368 codec + smart PA)
- [ ] Networking (Wi-Fi/BT connectivity stack), sensors, camera, and automation/CI hooks
- [ ] Deterministic record/replay and regression suites for every subsystem

## Building

Requirements: QEMU build dependencies (meson, ninja, glib, pixman, …) on Linux, macOS or MSYS2.

```bash
git clone --depth 1 -b xaga https://github.com/Cyber-Mobile/xaga-qemu.git xaga-qemu
cd xaga-qemu
scripts/build.sh            # configures build/ and builds qemu-system-aarch64
```

## Running

The emulator boots **your own device dumps** (none are distributed here):

- `fw/LUA_1` — preloader image
- `fw/gpt_512.bin` — GPT blob used by the BootROM stub
- `data/lu2.img` — raw UFS user-data LUN (sparse raw image recommended)

```bash
export MTK_MEM=8G MTK_ICOUNT=off MTK_LU2_SNAP=1
export MTK_NO_NAMETBL=1 MTK_KINFO_FLAG0=1 MTK_LU2_RO=1 MTK_WDT_STATUS_VAL=1
export MTK_NO_ATFLOG=1 MTK_NO_ATFARG=1 MTK_NO_ATFARG2=1 MTK_GZ_SKIP=1 MTK_NO_GTAPLANT=1

./build/qemu-system-aarch64 \
  -M mt6895,preloader=fw/LUA_1,gpt=fw/gpt_512.bin,data=data/lu2.img,entry-bytes=3072 \
  -m 8G -smp 1 -display none -monitor none -accel tcg,tb-size=256 \
  -serial file:OUT.log
```

### Useful environment switches

| Variable | Effect |
|---|---|
| `MTK_BOOT_MODE=2` | boot recovery |
| `MTK_FBDUMP=<prefix>` | dump composed frames as PPM |
| `MTK_DISP_SRC=simplefb\|drm` | select display path |
| `MTK_GCE_EXEC=on` | execute GCE/CMDQ instructions |
| `MTK_USB_HOSTMODE=1` | expose mtu3 as xHCI host with emulated devices |
| `MTK_USBDEV=1 MTK_USBDEV_TCP=4444` | device-mode UDC + TCP bridge for fastboot |
| `MTK_KCMDLINE_SUBST='old:new'` | same-length kernel cmdline substitution |
| `MTK_SMPDBG=1` | per-CPU debug trail |
| `MTK_ATF_LAZY_PLANT=1` | plant EL3 context before first non-PSCI SMC (secondary TEE) |

## Host-side fastboot client

`contrib/xaga-host/` contains a standalone Python client for the emulated UDC bridge:

```bash
python3 contrib/xaga-host/fastboot_cli.py --host 127.0.0.1 --port 4444 getvar version product current-slot
python3 contrib/xaga-host/fastboot_cli.py --host 127.0.0.1 --port 4444 --flash boot_a=boot.img
```

It implements USB enumeration, fastboot `download:`/`flash:` with 16 KiB chunking and a mock device
for offline testing (`contrib/xaga-host/lib/mock_device.py`).

## Repository layout

```
hw/arm/mt6895.c           SoC machine model and boot chain integration
hw/misc/mtk_*.c           syscon, IOMMU, GCE, UFS, SPMI, UDC, …
hw/display/mtk_disp.c     OVL/DSI display model
include/hw/arm/mt6895.h   machine defines
host/                     host-side fastboot client and mock device
docs/                     architecture and usage notes (images in docs/images)
scripts/                  build/run helpers
```

## Legal & provenance

- Contains **only original code and documentation**; no MediaTek/Xiaomi firmware, device dumps or
  ROM contents are distributed here. Bring your own dumps from a device you own.
- QEMU-derived files keep QEMU's license (**GPL-2.0**); new files are `GPL-2.0-or-later`.
- For research and development; not a tool for flashing real hardware.

## License

GPL-2.0 (see `LICENSE` / `COPYING`).
