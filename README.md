# xaga emulator (MT6895)

**A full-system, scriptable simulator for the Xiaomi xaga (Redmi Note 11T Pro / POCO X4 GT, MediaTek MT6895).**

Goal: boot the *real* device firmware stack — preloader → BL2_EXT → ATF → TEE → GZ → LK → Linux → Android —
and faithfully emulate the hardware around it, so that recovery, fastboot, adb and the Android UI run
against the emulated phone instead of a model of it.

This project is implemented as a QEMU machine model (`-M mt6895`) on top of
[QEMU](https://www.qemu.org/) (fork at branch `xaga`, based on `v11.1.1`). **QEMU is the substrate;
the emulator is the project.**

A hard rule shapes everything below: the model never patches guest images or plants guest state to get
past a check. Workarounds are allowed only as clearly named, default-off diagnostics, and every one of
them is listed in the injection-debt register and worked off.

![Recovery](docs/xaga/images/recovery-ui.png)

## What already works

| Area | State |
|---|---|
| BROM stub, preloader, BL2_EXT, ATF (TF-A v2.4) — real images, no image patching | ✅ |
| TEE (Soter450 / ISEE450, plaintext BL32) runs its own bootstrap and SiP path | ✅ |
| GZ (GenieZone hypervisor, EL2): `gz_a`/`gz_b` are decrypted **in the guest** by the DXCC AES model | 🧪 GZ executes and initialises itself; stops at a board-id ADC assertion (see roadmap) |
| LK → Linux 5.10 → vendor Android userspace (UI) | ✅ via the temporary GZ bypass described below |
| Recovery mode (`MTK_BOOT_MODE=2`) + Xiaomi recovery UI (full screen, no blit artifacts) | ✅ |
| Touchscreen: MTK SPI controller + Novatek NVT36672C (`MTK_TOUCH=1`) | ✅ driver binds as `input2`, real coordinates through EINT → GIC |
| PMIC/SPMI: MT6363 / MT6368 / MT6319, SPMI-PMIF, regulators and VSEL monitor aliases | ✅ all 15 main rails match the unit's own boot log |
| PMIC OTP/efuse windows and the fuse state the unit reports | ✅ mechanism; contents only where the unit's log documents them |
| Fuel gauge: bq28z610 (I2C) | ✅ |
| Display: OVL composition, dual-pipe half swap, MIPI DSI command queue, GCE/CMDQ execution, SimpleFB fallback | ✅ |
| Storage: UFS host controller (UIC/UTP/UCD), 4 KiB LUs, GPT, RPMB, block-backend (raw/snapshot/CoW) | ✅ |
| USB host: xHCI with keyboard / network / audio enumeration | ✅ |
| USB device mode: mtu3-style UDC + QMU, bridged to a host fastboot client (`getvar`, `download`, `flash`) | ✅ |
| Crypto engines: DXCC-SEC (AES-128-CTR, SHA-256) and MASP/HACC | ✅ execute the guest's descriptors |
| Secure address alias (`0x800000000 + PA`) for the TEE/GZ view, topckgen ULPOSC calibration, KPD, RTC | ✅ |
| SMP | 🚧 PSCI CPU_ON and CPC power-state ordering are modelled; the shipping run path uses `-smp 1` and 8-core bring-up is still open |
| Audio: AFE model + 440 Hz self-test | 🧪 model ready |

## Zero-injection status

"Zero injection" means: no guest-visible state is invented and no firmware image is patched. The boot
advances by modelling hardware correctly. Where that is not yet done, the model stops honestly instead
of faking the next step. With no `MTK_*` variables set at all, the ladder currently runs the entire
real stack into the Linux kernel:

```
BROM → preloader → BL2_EXT → ATF → TEE → GZ (EL2) → LK (fastboot/recovery platform) → Linux 5.10
```

The vendor kernel boots (`Linux version 5.10.269-g56de8342020a`, `Booting Linux on physical CPU 0x0`)
and stops in the power-domain path: `mtk-scpsys` fails to power on several domains (-110) and
`mtk-pd-chk-mt6895` raises `kernel BUG at drivers/clk/mediatek/mtk-pd-chk-mt6895.c:622` during the
first `genpd` power-off. That is the next piece of hardware to model (the SPM/SCPSYS status window the
vendor power-domain checker reads).

Two divergences from the unit are known and recorded: LK's RPMB check fails in the model (non-fatal,
the unit passes it), and `__apu_are_init` reports a SRAM1 FSM idle timeout the unit does not print.

To reach the UI today the project still uses **one named bypass**, `MTK_GZ_SKIP=1`, which redirects
ATF's BL32 hand-off straight to LK. It is default-off, it prints a line when it fires, and it exists
only while the GZ stage is being brought up. Everything else that used to be faked (ATF boot arguments,
the synthetic name table, the fabricated BL2_EXT entry, the BROM boot-argument block, PMIC register
values, the GZ ciphertext) has already been replaced by real models or real device data.

## Roadmap

- [x] Boot chain, display, UFS, USB, recovery UI
- [x] LK fastboot over emulated USB device mode (host-side client included)
- [x] LineageOS kernel boot to recovery (`MTK_PWR_STATE=1` power/clock/bus-protection modeling)
- [x] Real GZ decryption: DXCC-SEC AES engine executes the preloader's descriptors, `gz_a`/`gz_b` come out plaintext
- [x] Touchscreen end to end (driver bind, EINT, GIC, real coordinates)
- [x] PMIC rails matching the unit, OTP/efuse windows, ULPOSC calibration
- [x] Board-ID ADC model so GZ gets `cpu_type != 0` and the zero-injection boot passes GZ
- [x] Remove the synthetic ATF GTA tag (the model's own tag overlapped BL31's page tables and hung LK's CMDQ SMC)
- [ ] SPM/SCPSYS power-domain status so the kernel's power-domain checker stops tripping
- [ ] Remove the last named bypasses (GZ hand-off, EL3 plant, LK boot-arg plant)
- [ ] Full LineageOS/Android boot and `adb sideload` without any bypass
- [ ] Kernel USB gadget: `fastbootd`, `adb`, `adb sideload` in recovery
- [ ] SMP: 8 cores online with the TEE/SiP path working on every core
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

Recovery boot, the path that is exercised today (6 GiB, one core, deterministic timing):

```bash
export MTK_MEM=6G MTK_KINFO_FLAG0=1 MTK_WDT_STATUS_VAL=1 MTK_LU2_SNAP=1
export MTK_BOOT_MODE=2 MTK_PWR_STATE=1
export MTK_NO_ATFLOG=1 MTK_NO_GTAPLANT=1 MTK_NO_XHCI=1 MTK_GZ_SKIP=1   # see zero-injection status

./build/qemu-system-aarch64 \
  -M mt6895,preloader=fw/LUA_1,gpt=fw/gpt_512.bin,data=data/lu2.img,entry-bytes=3072 \
  -m 6G -smp 1 -display none -monitor none \
  -icount shift=6,sleep=off -accel tcg,tb-size=256 \
  -serial file:OUT.log
```

Drop `MTK_GZ_SKIP` (and the other diagnostics) to run the zero-injection ladder; it currently stops
inside GZ as described above. `-icount shift=4` trades timing fidelity for speed (~70 s to recovery on
a desktop CPU); `shift=6` is the slower, more conservative default used for comparisons.

### Useful environment switches

Boot and platform:

| Variable | Effect |
|---|---|
| `MTK_BOOT_MODE=2` | boot recovery |
| `MTK_PWR_STATE=1` | real power/clock/bus-protection sequencing (required by the LineageOS kernel) |
| `MTK_MEM=6G\|8G` | DRAM size presented to the guest |
| `MTK_LU2_SNAP=1` | user-data LUN snapshot mode |
| `MTK_TOUCH=1` | instantiate the MTK SPI controller and the Novatek NVT36672C touch client (`0`/unset = off) |

Display and input:

| Variable | Effect |
|---|---|
| `MTK_FBDUMP=<prefix>` | dump composed frames as PPM |
| `MTK_DISP_SRC=simplefb\|drm` | select display path |
| `MTK_GCE_EXEC=on` | execute GCE/CMDQ instructions |
| `MTK_DISP_VBLANK=0` | **A/B:** disable the default-on DISP mutex frame-EOF model |
| `MTK_NO_DSI_MODEL=1` | **A/B:** disable the DSI controller model |
| `MTK_NO_DISPMUTEX=1` | **A/B:** disable the DISP mutex acquire/EOF model |
| `MTK_BL_LOG=1` | log KTZ8863A backlight/bias updates |

USB and memory:

| Variable | Effect |
|---|---|
| `MTK_USB_HOSTMODE=1` | expose mtu3 as an xHCI host with emulated devices |
| `MTK_USBDEV=1 MTK_USBDEV_TCP=4444` | device-mode UDC + TCP bridge for fastboot/adb |
| `MTK_IOMMU=0` | disable the **default-on** M4U/IOMMU translation and let clients program physical addresses |
| `MTK_IOMMU_LOG=1` | trace the M4U page-table base, TLB invalidations and translations |
| `MTK_DT_NOIOMMU=1` | **deprecated** legacy bring-up hack (NOPs the DT `iommus` properties) |
| `MTK_KCMDLINE_SUBST='old:new'` | same-length kernel cmdline substitution (diagnostic) |

Diagnostics (observation only; none of them change guest-visible behaviour):

| Variable | Effect |
|---|---|
| `MTK_EFUSE_TRACE=1`, `MTK_EFUSE_STATE=xaga\|blank`, `MTK_EFUSE_FILE=<path>` | efuse/OTP contents and access trace |
| `MTK_DXCCLOG=1`, `MTK_DXCC_AESLOG=1`, `MTK_GZ_DUMP=<prefix>` | DXCC-SEC register/descriptor trace and buffer dumps |
| `MTK_SMPDBG=1` | per-CPU debug trail |

Named bypasses (default off; listed in the debt register, being removed):

| Variable | Effect |
|---|---|
| `MTK_GZ_SKIP=1` | redirect ATF's BL32 hand-off to LK instead of GenieZone |
| `MTK_NO_GTAPLANT=1` | disable the ATF/GTA EL3 plant |
| `MTK_ATF_LAZY_PLANT=1` | plant the EL3 context before the first non-PSCI SMC |

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
hw/misc/mtk_*.c            syscon/efuse, IOMMU, GCE, UFS, SPMI-PMIC, EINT, UDC, KPD, RTC, …
hw/ssi/mtk_spi.c           MTK SPI controller (touch bus)
hw/i2c/mtk_i2c.c           I2C controller (fuel gauge and friends)
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
