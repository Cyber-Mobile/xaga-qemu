# Host-side tools

Host-side helper for the emulated USB device-mode bridge of the xaga emulator.

The QEMU machine model can expose a **virtual USB device controller (UDC)** over a TCP chardev.
This directory contains:

- a reusable **virtual USB host** library (enumeration, control transfers, bulk endpoints),
- a **fastboot protocol client** (getvar / download / flash / reboot / oem),
- a **mock device** for offline unit tests,
- an **adb protocol skeleton** for future recovery work.

Requirements: Python 3.9+, standard library only (no third-party packages).

## Frame protocol

The QEMU UDC model speaks a simple length-prefixed protocol over TCP:

    frame := u8 type | u16le payload_len | payload

Host -> device:

| type | name | payload |
|---|---|---|
| `0x01` | SETUP | 8-byte USB setup packet |
| `0x02` | EP1_OUT | fastboot command or download chunk (16 KiB) |
| `0x03` | EP0_ACK | control-IN completion acknowledgement |
| `0x10` | HOST_CONNECT | bus reset / connect |
| `0x11` | HOST_DISCONNECT | disconnect |

Device -> host:

| type | name | payload |
|---|---|---|
| `0x81` | EP0_IN | control-IN data |
| `0x82` | EP1_IN | bulk IN data (fastboot reply) |
| `0x83` | EP1_ACK | optional per-GPD consumption acknowledgement |
| `0x90` | LOG | ASCII status line (`READY`, `SOFT_CONN`, `QMU_READY`, …) |

## Library

| module | purpose |
|---|---|
| `lib/vusb.py` | connection handling, frame codec, USB enumeration, control and bulk transfers |
| `lib/fastboot.py` | fastboot client: bring-up sequencing, getvar, download, flash, reboot, oem |
| `lib/mock_device.py` | mock UDC server implementing a fastboot device (and a minimal adb device) |
| `lib/adb.py` | adb packet codec and basic CNXN/OPEN/WRTE helpers |

## CLI

```bash
python3 fastboot_cli.py --host 127.0.0.1 --port 4444 getvar version product current-slot
python3 fastboot_cli.py --host 127.0.0.1 --port 4444 --flash boot_a=boot.img
python3 fastboot_cli.py --host 127.0.0.1 --port 4444 download boot.img
python3 fastboot_cli.py --host 127.0.0.1 --port 4444 reboot recovery
```

Useful options:

| option | default | meaning |
|---|---|---|
| `--chunk-size` | 16384 | EP1 OUT chunk size (bytes) |
| `--chunk-delay` | 0.002 | delay between chunks (the UDC has no per-chunk ACK; raise it if a download times out) |
| `--pacing` | delay | `delay` (fixed pacing) or `ack` (credit-based, requires the 0x83 ACK frame) |
| `--download-timeout` / `--flash-timeout` | 120 / 180 | reply timeouts (seconds) |
| `--log FILE` | – | write a diagnostic log |

## Testing without QEMU

```bash
cd contrib/xaga-host
python3 -m unittest discover -s tests
```

The mock device implements a complete fastboot device, so the enumeration, download and flash
paths can be tested offline. `tests/bench_mock.py 64` measures the 64 MiB transfer path.

## Connecting to the emulator

1. Start QEMU with the UDC model enabled:
   `MTK_USBDEV=1 MTK_USBDEV_TCP=4444` plus the normal boot environment.
2. Make LK enter fastboot (for example `MTK_BOOT_MODE=0 MTK_RGU_KEY=3`).
3. Run the CLI against the TCP port. The client waits for the `SOFT_CONN` log line,
   performs USB enumeration, waits for `QMU_READY` and then sends fastboot commands.

Notes:

- A client that connects late (after LK has already enumerated) falls back to probing enumeration;
  the fastest path is to start the client before the device reaches fastboot.
- 16 KiB chunks and the default 2 ms pacing work with the current UDC model; the optional 0x83
  acknowledgement frame allows tighter credit-based pacing.
- Replies larger than the device-side TX buffer are truncated; prefer `getvar` over `getvar:all`.

## License

GPL-2.0-or-later.
