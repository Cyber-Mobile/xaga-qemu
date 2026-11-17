# LK FastBoot over the emulated USB device-mode bridge

1. Start QEMU with `MTK_USBDEV=1` and `MTK_BOOT_MODE=0 MTK_RGU_KEY=3` (the RGU key latch makes LK
   enter fastboot instead of booting Linux).
2. Connect the host client to the UDC TCP port (`MTK_USBDEV_TCP`, default 4444):
   `python3 contrib/xaga-host/fastboot_cli.py --host 127.0.0.1 --port 4444 getvar version`
3. The client performs USB enumeration, then speaks the fastboot protocol: `getvar:`, `download:<hex>`, `flash:<partition>`, `reboot-*`.

Notes:
- 16 KiB chunks; the UDC has no per-chunk ACK, keep the default 2 ms pacing.
- `MTK_FLASH_RW=1` opens LUN0 as a snapshot (writes are discarded when QEMU exits).
- To flash persistently, boot from a **copy** of your data image and leave `MTK_LU2_RO`/`MTK_LU2_SNAP` unset.
