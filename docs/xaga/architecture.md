# Architecture overview

`-M mt6895` is a single machine model that covers the whole MT6895 SoC surface needed to run the
vendor firmware stack. This document is a map of the model.

## Boot chain

- **BootROM stub**: executes the on-chip SRAM payload, provides block-read helpers backed by the
  `preloader=` image, and hands control to the preloader.
- **Preloader**: real MediaTek preloader runs from the supplied `LUA_1` image; the model feeds it
  storage, RTC, PMIC and DRAM calibration responses.
- **BL2_EXT / ATF / TEE**: real images; the model provides EL3 state ownership and a warm-reset
  path. ATF is patched in-memory only where the emulated SoC diverges from silicon.
- **LK**: runs from DRAM (`0x50f00000`), boots with either normal, recovery (`MTK_BOOT_MODE=2`) or
  fastboot (`MTK_RGU_KEY=3`, a one-shot RGU key latch) mode.
- **Linux**: vendor 5.10 kernel takes over the same hardware model; GZ is replaced by a stub.

## Subsystems

| Model | File | Notes |
|---|---|---|
| Main SoC glue, windows, IRQ wiring | `hw/arm/mt6895.c` | environment-gated workarounds, FDT injections |
| Syscon / clock-controller windows | `hw/misc/mtk_syscon.c` | register files, gate/clock acknowledgements |
| Power domains | `hw/misc/mtk_scpsys.c` | SPM/SCPSYS power on/off state |
| GCE / CMDQ | `hw/misc/mtk_gce.c` | instruction engine, address translation, mask semantics |
| IOMMU | `hw/misc/mtk_iommu.c` | TLB invalidation completion, fault status |
| UFS | `hw/misc/mtk_ufs{c,lu,hci}.c` | UIC/UTP/UCD SCSI translation, 4 KiB LUs, RPMB |
| Display | `hw/display/mtk_disp.c` | OVL layers, dual-pipe split/swap, DSI cmd queue, frame dumps |
| USB host | `hw/usb/hcd-xhci.c` changes | sysbus xHCI wiring + slot count fix |
| USB device | `hw/misc/mtk_usbdev.c` + `mtk_usb_glue.c` | mtu3-style UDC, QMU, TCP chardev bridge |
| Keypad / PMIC / RTC | `hw/misc/mtk_kpd.c`, `mtk_spmi_pmif.c` | KPD, SPMI PMIF, RTC registers |
| Audio | `hw/audio/mtk_afe.c` | AFE memory-mapped model + self-test device |
| MSDC / UART / I2C / timers | `hw/sd/`, `hw/char/`, `hw/i2c/`, `hw/timer/` | peripheral models |

## USB device-mode fastboot bridge

`MTK_USBDEV=1` instantiates the UDC model and exposes a TCP chardev (`MTK_USBDEV_TCP`). The host
client (`contrib/xaga-host/`) implements USB enumeration and the fastboot protocol. Endpoint/frame protocol:
`0x01` SETUP, `0x02` EP1 OUT, `0x10` CONNECT from host; `0x81` EP0 IN, `0x82` EP1 IN, `0x90` log
to host. QMU rings use four 16-byte GPDs with the wrap semantics of the real controller.
