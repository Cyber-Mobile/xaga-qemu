# Host-side tools

Host-side helper for the emulated USB device-mode bridge of the xaga emulator.

The QEMU machine model can expose a **virtual USB device controller (UDC)** over a TCP chardev.
This directory contains:

- a reusable **virtual USB host** library (enumeration, control transfers, bulk endpoints),
- a **fastboot protocol client** (getvar / download / flash / reboot / oem),
- a **mock device** for offline unit tests,
- an **adb client and CLI** (CNXN/AUTH, shell, reboot, sideload, push) for recovery work.

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
| `lib/adb.py` | adb client: packet codec, CNXN/AUTH (RSA-SHA1), stream multiplexing, shell v2/legacy, reboot, sideload-host/legacy, sync push |
| `adb_cli.py` | adb command line: devices / shell / sideload / reboot / push |

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


## adb CLI (recovery shell / sideload)

`adb_cli.py` speaks adb over the same chardev transport (24-byte packets;
CNXN/AUTH/OPEN/WRTE/OKAY/CLSE streams).  It targets the recovery adbd that the
kernel USB gadget model will expose; the same client is covered end-to-end by
the mock adb gadget tests, so it can be validated before QEMU is ready.

`@bash
python3 adb_cli.py --host 127.0.0.1 --port 4444 devices
python3 adb_cli.py --port 4444 shell getprop ro.product.device
python3 adb_cli.py --port 4444 reboot recovery
python3 adb_cli.py --port 4444 push local.bin /data/local/tmp/local.bin
python3 adb_cli.py --port 4444 sideload lineage-20-xaga-signed.zip
`@

Useful options:

| option | default | meaning |
|---|---|---|
| `--key FILE` | `$ADBKEY` or `~/.android/adbkey` | RSA host key; auto-generated with `openssl genrsa` if missing |
| `--timeout` | 30 | per-operation timeout (seconds); raise it for large sideloads |
| `--block-size` | 65536 | sideload-host block size advertised to minadbd |
| `--no-legacy-fallback` | off | fail instead of using the legacy `sideload:` push service |
| `--maxdata` | 4096 | adb transport packet payload limit |
| `-q` / `-v` | - | quiet progress / verbose protocol logs |

Sideload protocol (what the implementation does):

- ADB handshake: `CNXN` with `host::features=cmd,shell_v2,stat_v2`; if the
  device answers `AUTH(TOKEN)` the client signs the 20-byte token with RSA-SHA1
  (PKCS#1 v1.5) and, when the key is unknown to the device, uploads
  `AUTH(RSAPUBLICKEY)` in Android's 524-byte little-endian key format.
- Open the service `sideload-host:<file-size>:<block-size>` (what current
  LineageOS/AOSP recovery `minadbd` expects; the legacy `sideload:<size>`
  service only exists for pre-KitKat devices and is used as a fallback).
- minadbd sends 8-byte block requests (zero-padded decimal block index) as adb
  stream data; the host answers with the matching file block (the last block is
  short).  The package is read with `seek()/read()`, so multi-GB images are
  never loaded into memory.
- When the package has been served the device answers `DONEDONE` (or
  `FAILFAIL`); the CLI prints `Total xfer: <ratio>x` and exits 0/1.
- A typical OTA is read roughly twice (verification + install), so the AOSP
  progress estimate is `xfer * 47 / size` (100% at about 2.13x the package
  size); the CLI shows both the estimate and the real byte counts.

Auth notes: the RSA signer and the Android public-key encoder are implemented
with the standard library (minimal DER parser + modular exponentiation);
`openssl` is only used to generate a new key when none exists, or as a signing
fallback for key formats the pure-Python parser does not understand.


## Testing without QEMU

```bash
cd contrib/xaga-host
python3 -m unittest discover -s tests
```

The mock device implements a complete fastboot device and an adb gadget (CNXN/AUTH,
shell v1/v2, reboot, sideload-host/legacy and sync push), so enumeration, flashing
and sideload paths can be tested offline:

`@bash
cd contrib/xaga-host
python3 -m unittest discover -s tests                  # 107 tests, roughly 37 s
VUSB_BIG_TEST=1 python3 -m unittest discover -s tests  # add 64 MiB fastboot + sideload
python3 tests/bench_mock.py 64                         # fastboot path throughput
`@

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
- adb sideload needs the guest adbd (kernel USB gadget model) to expose the adb
  interface; recovery adbd usually runs with authentication disabled, but the
  AUTH path is implemented for regular adbd builds.
- NOT implemented yet in the adb client: sync pull (RECV), stat/list, shell_v2
  stdin streaming and incremental install; see the TODO block in `lib/adb.py`.
- The sideload 64 MiB regression is gated behind `VUSB_BIG_TEST=1` like the
  fastboot big-image test; regular runs stay under a minute.

## License

GPL-2.0-or-later.
