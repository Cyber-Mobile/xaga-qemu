# xaga emulator (MT6895)

**A full-system, scriptable simulator for the Xiaomi xaga (Redmi Note 11T Pro / POCO X4 GT, MediaTek MT6895).**

Goal: boot the *real* device firmware stack — preloader → BL2_EXT → ATF → TEE → GZ → LK → Linux → Android —
and faithfully emulate the hardware around it, so that recovery, fastboot, adb and the Android UI run
against the emulated phone instead of a model of it.

This project is implemented as a QEMU machine model (`-M mt6895`) on top of
[QEMU](https://www.qemu.org/) (fork at branch `xaga`, based on `v11.1.1`). **QEMU is the substrate;
the emulator is the project.**

A hard rule shapes everything below: the model never patches guest images or plants guest state to get
past a check. Missing hardware behaviour is modelled as hardware behaviour; workarounds are allowed only
as clearly named, default-off diagnostics.

![Recovery](docs/xaga/images/recovery-ui.png)

## What already works

| Area | State |
|---|---|
| BROM stub, preloader, BL2_EXT, ATF (TF-A v2.4) — real images, no image patching | ✅ |
| TEE (Soter450 / ISEE450, plaintext BL32) runs its own bootstrap and SiP path | ✅ |
| GZ (GenieZone hypervisor, EL2): `gz_a`/`gz_b` decrypted **in the guest** by the DXCC AES model | ✅ runs, registers its memory-protection protocol, hands off to Linux |
| LK → Linux 5.10 → Android first-stage init → second-stage `init` → `post-fs` | ✅ |
| 8-core SMP bring-up (MPIDR.MT, PSCI conduit `disabled`, CPC / power-state ordering) | ✅ `8 CPUs` online and activated on the normal-boot path |
| Recovery mode (`MTK_BOOT_MODE=2`) + Xiaomi recovery UI (full screen, no blit artifacts) | ✅ |
| Touchscreen: MTK SPI controller + Novatek NVT36672C | ✅ driver binds as `input2`, real coordinates through EINT → GIC; in-guest `getevent` sees DOWN/UP |
| USB device mode: mtu3-style MAC/QMU UDC, native Type-C/TCPC chain (MT6375 → BC1.2 → extcon → role switch) | ✅ enumeration, `A_CNXN`, `shell_v2`, `getevent` in Recovery — **no `MTK_USB_GADGET`** |
| `sideload-host` (Apply update from ADB) | 🚧 enumerates and answers `sideload-host:...`, but `adbd` still owns FunctionFS (`verifiedbootstate=green`) |
| Display: OVL composition, dual-pipe half swap, MIPI DSI command queue, GCE/CMDQ execution, SimpleFB fallback | ✅ Recovery path; normal-boot UI still shows the LK logo |
| Storage: UFS host controller (UIC/UTP/UCD), 4 KiB LUs, GPT, RPMB, block-backend (raw/snapshot/CoW) | ✅ |
| USB host: xHCI with keyboard / network / audio enumeration | ✅ |
| USB device + host-side fastboot client (`getvar`, `download`, `flash`) | ✅ |
| PMIC/SPMI: MT6363 / MT6368 / MT6319, SPMI-PMIF, regulators, VSEL monitor aliases | ✅ all 15 main rails match the unit's own boot log |
| MT6375 sub-PMIC: TCPC/BC1.2, ADC, IRQ; I²C APDMA; EINT level-latched IRQs | ✅ |
| GPIO/pinctrl: MT6895 controller register banks (data / SET / CLR / MODE MWR) | ✅ |
| Crypto engines: DXCC-SEC (AES-128-CTR, SHA-256) and MASP/HACC | ✅ execute the guest's descriptors |
| Secure address alias (`0x800000000 + PA`) for the TEE/GZ view, topckgen ULPOSC control, KPD, RTC | ✅ (ULPOSC calibration meter still being brought up) |
| Audio: AFE model + 440 Hz self-test | 🧪 model ready |

## Where the zero-injection boot stands

With **no `MTK_*` variables set at all** the ladder runs the entire real stack into Android:

```
BROM → preloader → BL2_EXT → ATF → TEE → GZ (EL2) → LK → Linux 5.10.269 → init (1st stage)
     → dynamic partitions mounted → init (2nd stage) → servicemanager / vold / keystore2 → post-fs
```

Measured on the 8-core path: all eight CPUs come online and are activated, the seven dynamic
partitions mount, `/bootstrap-apex/apex-info-list.xml` loads, `/metadata` is fsck'd and mounted, and
`init` reaches `post-fs` and starts the vendor module loader.

The current stop is inside the vendor SCP module: `scp.ko`'s `init` clears the SCP exception-dump
window and takes a data abort on that region, which the kernel reports as `bad mode` and panics. The
GPIO gate that used to wedge `modprobe scp` (the `gpio-vreq` mode check) now passes — the vendor driver
prints `v_req muxpin setting is correct` — so the next pieces of hardware to model are the SCP dump
window and the ULPOSC calibration meter that `scp_dvfs` reads.

Known divergences from the unit: LK's RPMB check fails in the model (non-fatal; the unit passes it), and
`__apu_are_init` reports an SRAM1 FSM idle timeout the unit does not print.

## Roadmap

- [x] Boot chain, display, UFS, USB, recovery UI
- [x] LK fastboot over emulated USB device mode (host-side client included)
- [x] LineageOS kernel boot to recovery (power/clock/bus-protection modelling)
- [x] Real GZ decryption: DXCC-SEC AES engine executes the preloader's descriptors
- [x] Touchscreen end to end (driver bind, EINT, GIC, real coordinates)
- [x] PMIC rails matching the unit, OTP/efuse windows, ULPOSC control
- [x] 8-core SMP on the default (zero-injection) normal-boot path
- [x] Native USB/ADB in Recovery: enumeration, `A_CNXN`, `shell_v2`, in-guest `getevent`
- [x] MT6375 TCPC/BC1.2 + I²C APDMA + EINT level IRQ; USB QMU stop/restart
- [x] MT6895 GPIO controller semantics (SET/CLR/MODE masked write)
- [ ] SCP exception-dump window + ULPOSC calibration meter (current blocker)
- [ ] Normal-boot UI: `zygote`, `SurfaceFlinger`, launcher
- [ ] `sideload-host` end to end (unlock chain / `minadbd` taking FunctionFS)
- [ ] Full LineageOS/Android boot without any diagnostic env
- [ ] Audio path end-to-end (AFE + MT6368 codec + smart PA)
- [ ] Networking (Wi-Fi/BT connectivity stack), sensors, camera
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

Zero-injection normal boot, 8 cores, deterministic timing (the path used for bring-up):

```bash
./build/qemu-system-aarch64 \
  -M mt6895,preloader=fw/LUA_1,gpt=fw/gpt_512.bin,data=data/lu2.img,entry-bytes=3072 \
  -m 6G -smp 8 -display none -monitor none \
  -icount shift=4,sleep=off -accel tcg,tb-size=4096 \
  -serial file:OUT.log
```

Recovery, with the touch panel and USB device mode enabled:

```bash
MTK_BOOT_MODE=2 MTK_TOUCH=1 MTK_USBDEV=1 \
./build/qemu-system-aarch64 \
  -M mt6895,preloader=fw/LUA_1,gpt=fw/gpt_512.bin,data=data/lu2.img,entry-bytes=3072 \
  -m 6G -smp 8 -display none -monitor none \
  -icount shift=4,sleep=off -accel tcg,tb-size=4096 \
  -serial file:OUT.log
```

The kernel's console is muted once it takes over; the model can periodically scan the kernel log ring for
markers with `MTK_KMEM_SCAN` / `MTK_KMEM_MS` (see below).

### Environment switches

Platform:

| Variable | Effect |
|---|---|
| `MTK_BOOT_MODE=2` | boot recovery |
| `MTK_LU2_SNAP=1` | user-data LUN snapshot mode (writes stay in a scratch overlay) |
| `MTK_LU2_RO=1` | user-data LUN read-only |
| `MTK_TOUCH=1` | instantiate the MTK SPI controller and the Novatek NVT36672C touch client |
| `MTK_USBDEV=1` | device-mode UDC (mtu3-style) for fastboot/adb |
| `MTK_USBDEV_TCP=4444` | bridge that UDC to a TCP chardev (host client connects to `127.0.0.1:4444`) |
| `MTK_USB_HOSTMODE=1` | expose mtu3 as an xHCI host with emulated devices |
| `MTK_PWR_STATE=1` | power/clock/bus-protection sequencing (required by the LineageOS kernel) |
| `MTK_KMEM_SCAN`, `MTK_KMEM_MS` | scan the kernel log ring for the configured markers at the given host-time windows |

Observation only (none change guest-visible behaviour):

| Variable | Effect |
|---|---|
| `MTK_FBDUMP=<prefix>`, `MTK_FBDUMP_MS` | dump composed frames as PPM |
| `MTK_DISP_SRC` (`simplefb` or `drm`) | select the display path |
| `MTK_GCE_EXEC=on` | execute GCE/CMDQ instructions |
| `MTK_DXCCLOG=1`, `MTK_DXCC_AESLOG=1`, `MTK_GZ_DUMP=<prefix>` | DXCC-SEC register/descriptor trace and buffer dumps |
| `MTK_IOMMU_LOG=1` | M4U page-table bases, TLB invalidations and translations |
| `MTK_SYSCON_TRACE=1`, `MTK_PINCTRL_RD=1` | syscon / pinctrl access traces |
| `MTK_INIT_CRASH_TRACE=1` | keep the init crash trail |
| `MTK_SMPDBG=1` | per-CPU bring-up trail |

## Testing

QTest suites for the machine model (each instantiates `-machine mt6895` and drives registers directly):

```bash
ninja -C build tests/qtest/mtk-miu-test tests/qtest/mtk-pl-sram-test \
      tests/qtest/mtk-mpidr-test tests/qtest/mtk-mt6375-test \
      tests/qtest/mtk-usbdev-test tests/qtest/mtk-pinctrl-test
QTEST_QEMU_BINARY=build/qemu-system-aarch64 build/tests/qtest/mtk-pinctrl-test
```

Current suites: MIU range readback (2), PL SRAM neighbour isolation (1), MPIDR board profile (1),
MT6375/TCPCI/I²C-APDMA/EINT (2), USB QMU stop/restart (1), GPIO bank semantics (4).

## Host-side fastboot / adb client

`contrib/xaga-host/` contains a standalone Python client for the emulated UDC bridge:

```bash
python3 contrib/xaga-host/fastboot_cli.py --host 127.0.0.1 --port 4444 getvar version product current-slot
python3 contrib/xaga-host/fastboot_cli.py --host 127.0.0.1 --port 4444 --flash boot_a=boot.img
```

It implements USB enumeration, fastboot `download:`/`flash:` with 16 KiB chunking, an adb client
(CNXN/AUTH, shell, reboot, sideload, push) and a mock device for offline testing
(`contrib/xaga-host/lib/mock_device.py`).

## Repository layout

```
hw/arm/mt6895.c            SoC machine model and boot-chain integration
hw/misc/mtk_*.c            syscon/efuse, IOMMU, GCE, UFS, SPMI-PMIC, EINT, UDC, KPD, RTC, USB device, …
hw/ssi/mtk_spi.c           MTK SPI controller (touch bus)
hw/i2c/mtk_i2c.c           I2C controller + APDMA (fuel gauge, MT6375)
hw/i2c/mt6375.c            MT6375 sub-PMIC / TCPC / BC1.2 / ADC
hw/display/mtk_disp.c      OVL/DSI display model
include/hw/arm/mt6895.h    machine defines and register maps
contrib/xaga-host/         host-side fastboot/adb client and mock device
docs/xaga/                 architecture and usage notes (images in docs/xaga/images)
scripts/                   build/run helpers
```

## Legal & provenance

- Contains **only original code and documentation**; no MediaTek/Xiaomi firmware, device dumps or
  ROM contents are distributed here. Bring your own dumps from a device you own.
- QEMU-derived files keep QEMU's license (**GPL-2.0**); new files are `GPL-2.0-or-later`.
- For research and development; not a tool for flashing real hardware.

## License

GPL-2.0 (see `LICENSE` / `COPYING`).
