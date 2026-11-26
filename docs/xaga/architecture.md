# Architecture overview

`-M mt6895` is a single machine model that covers the whole MT6895 SoC surface needed to run the
vendor firmware stack. This document is a map of the model.

## Boot chain

- **BootROM stub**: executes the on-chip SRAM payload, provides block-read/write helpers backed by the
  `preloader=` image, and hands control to the preloader.
- **Preloader**: real MediaTek preloader runs from the supplied `LUA_1` image; the model feeds it
  storage, RTC, PMIC/SPMI, efuse/OTP, AUXADC and DRAM calibration responses.
- **BL2_EXT / ATF**: real images. ATF is TF-A v2.4, loaded at `0x48200000`; the model owns EL3's
  machine state but does not rewrite the image.
- **TEE (BL32)**: the real Soter450 / ISEE450 image, plaintext, enters at EL1 through the genuine
  BL2/ATF hand-off and issues its SiP calls to ATF.
- **GZ (GenieZone)**: the real hypervisor payload, AES-128-CTR scrambled. The preloader hands the
  DXCC-SEC engine a key and counter through its own descriptors; the model executes that AES stream,
  so the plaintext appears in DRAM and the TEE can ERET into GZ at EL2 through the secure alias
  `0x800000000 + PA`.
- **LK**: runs from DRAM (`0x50f00000`), boots with either normal, recovery (`MTK_BOOT_MODE=2`) or
  fastboot (`MTK_RGU_KEY=3`, a one-shot RGU key latch) mode.
- **Linux**: vendor 5.10 kernel takes over the same hardware model.

## Subsystems

| Model | File | Notes |
|---|---|---|
| Main SoC glue, windows, IRQ wiring | `hw/arm/mt6895.c` | BootROM stub, boot-chain integration, DXCC-SEC (AES/SHA), MASP/HACC, secure alias, topckgen, DSU PMU system registers, DebugTop/CONSYS MMIO windows |
| GICv3 interrupt controller | `hw/intc/arm_gicv3*.c` | ARM GICv3 per-group HPPI tracking (`hppi_grp[3]`) so disabled G1NS interrupts do not mask Secure PPIs |
| Syscon / efuse / clock-controller windows | `hw/misc/mtk_syscon.c` | register files, OTP word map, gate/clock acknowledgements |
| SPMI PMIC (MT6363/MT6368/MT6319) | `hw/misc/mtk_spmi_pmif.c` | PMIF, regulators, VSEL monitor aliases, OTP windows |
| Power domains | `hw/misc/mtk_scpsys.c` | SPM/SCPSYS power on/off state, CPC sequencing |
| Tinysys SCMI v2 mailbox | `hw/misc/mtk_tinysys_mbox.c` | DEN0056 Base (`0x10`) + vendor (`0x80`) protocols with in-place A2P shared-memory replies |
| ADSP subsystem & 4-channel mailbox | `hw/misc/mtk_adsp.c` | CFG/CFG2 control windows, MBOX0..3 (GIC SPI `644..647`), dual-core DTCM |
| GPUEB mailbox & gpufreq IPI | `hw/misc/mtk_gpueb.c` | RV33 boot handshake, 160-slot raw-payload `mbox0` SRAM, GIC SPI `516`, `IPI_ID_GPUFREQ` responder |
| GCE / CMDQ | `hw/misc/mtk_gce.c` | instruction engine, address translation, JUMPC/LOGIC semantics |
| IOMMU | `hw/misc/mtk_iommu.c` | 34-bit ARMv7 short-descriptor + `MTK_EXT` page-table translation, TLB invalidation completion, fault status |
| UFS | `hw/misc/mtk_ufs{c,lu,hci}.c` | UIC/UTP/UCD SCSI translation, 4 KiB LUs, RPMB |
| Display | `hw/display/mtk_disp.c` | OVL layers, dual-pipe split/swap, DSI cmd queue, frame dumps |
| Interrupt router | `hw/misc/mtk_eint.c` | EINT mux from the pinctrl cells the DTB describes |
| SPI (touch bus) | `hw/ssi/mtk_spi.c`, `hw/ssi/nvt36672c.c` | MTK SPI controller + Novatek NVT36672C client |
| I2C devices | `hw/i2c/mtk_i2c.c`, `hw/i2c/bq28z610.c`, `hw/i2c/mt6375.c` | I²C controller + APDMA, fuel gauge, MT6375 sub-PMIC / TCPC / BC1.2 |
| USB host | `hw/usb/hcd-xhci.c` changes | sysbus xHCI wiring + slot count fix |
| USB device | `hw/misc/mtk_usbdev.c` + `mtk_usb_glue.c` | mtu3-style UDC, QMU, TCP chardev bridge |
| Keypad / RTC | `hw/misc/mtk_kpd.c`, RTC windows | KPD, RTC registers |
| Audio | `hw/audio/mtk_afe.c` | AFE memory-mapped model + self-test device |
| MSDC / UART / timers | `hw/sd/`, `hw/char/`, `hw/timer/` | peripheral models |

## Injection policy

The model never patches guest images and never invents guest-visible state to pass a check. The
default normal-boot path runs natively with zero `MTK_*` overrides; where hardware behaviour is
not yet complete, missing peripherals are modelled as hardware rather than bypassed in the boot
chain.

## USB device-mode fastboot bridge

`MTK_USBDEV=1` instantiates the UDC model and exposes a TCP chardev (`MTK_USBDEV_TCP`). The host
client (`contrib/xaga-host/`) implements USB enumeration and the fastboot protocol. Endpoint/frame
protocol: `0x01` SETUP, `0x02` EP1 OUT, `0x10` CONNECT from host; `0x81` EP0 IN, `0x82` EP1 IN,
`0x90` log to host. QMU rings use four 16-byte GPDs with the wrap semantics of the real controller.
