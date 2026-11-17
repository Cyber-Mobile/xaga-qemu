#!/usr/bin/env python3
"""fastboot CLI for the MT6895 QEMU virtual UDC.

Examples
--------
    # device information
    python3 fastboot_cli.py --host 127.0.0.1 --port 4444 getvar version
    python3 fastboot_cli.py --host 127.0.0.1 --port 4444 getvar product current-slot

    # flash a local image
    python3 fastboot_cli.py --host 127.0.0.1 --port 4444 \
        --flash boot=boot.img

    # raw download / partition flash / reboot / oem passthrough
    python3 fastboot_cli.py --port 4444 download boot.img
    python3 fastboot_cli.py --port 4444 flash boot boot.img
    python3 fastboot_cli.py --port 4444 reboot bootloader
    python3 fastboot_cli.py --port 4444 oem device-info

The tool prints progress and per-phase timing to stderr and the result lines
to stdout.  Exit code is 0 on success, 1 on a fastboot/transport failure and
2 on a command line error.
"""
from __future__ import annotations

import argparse
import logging
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from lib import fastboot, vusb  # noqa: E402

MIB = 1024.0 * 1024.0
CLI_VERSION = "1.1.0"


def human_bytes(count: int) -> str:
    count = float(count)
    if count >= MIB:
        return "%.1f MiB" % (count / MIB)
    if count >= 1024.0:
        return "%.1f KiB" % (count / 1024.0)
    return "%d B" % int(count)


def human_rate(byte_count: int, seconds: float) -> str:
    if seconds <= 0:
        return "-"
    return "%.1f MiB/s" % (byte_count / seconds / MIB)


class ProgressPrinter:
    """Print progress on stderr every *step* percent (and at the end)."""

    def __init__(self, label: str, total: int, *, quiet: bool = False, step: float = 10.0) -> None:
        self.label = label
        self.total = int(total)
        self.quiet = quiet
        self.step = float(step)
        self._last_reported = -1.0

    def __call__(self, done: int, total: int, elapsed: float) -> None:
        if self.quiet or total <= 0:
            return
        percent = 100.0 * done / total
        finished = done >= total
        if not finished and percent - self._last_reported < self.step:
            return
        self._last_reported = percent
        sys.stderr.write(
            "[%s] %3.0f%% %s / %s  %s\n"
            % (self.label, percent, human_bytes(done), human_bytes(total),
               human_rate(done, elapsed))
        )
        sys.stderr.flush()


def parse_flash_spec(spec: str):
    if "=" not in spec:
        raise ValueError("--flash expects PART=PATH, got %r" % (spec,))
    partition, path = spec.split("=", 1)
    partition = partition.strip()
    path = path.strip()
    if not partition or not path:
        raise ValueError("--flash expects PART=PATH, got %r" % (spec,))
    return partition, path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="fastboot_cli.py",
        description="fastboot client for the MT6895 QEMU virtual UDC (LK fastboot).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--host", default="127.0.0.1",
                        help="QEMU chardev host (default 127.0.0.1)")
    parser.add_argument("--port", type=int, default=4444,
                        help="QEMU chardev TCP port (MTK_USBDEV_TCP, default 4444)")
    parser.add_argument("--version", action="version", version="%(prog)s " + CLI_VERSION)

    tuning = parser.add_argument_group("bring-up / transfer tuning")
    tuning.add_argument("--connect-timeout", type=float, default=30.0,
                        help="seconds to retry the chardev TCP connect")
    tuning.add_argument("--soft-conn-timeout", type=float, default=150.0,
                        help="seconds to wait for LK's U2 pull-up (LOG SOFT_CONN)")
    tuning.add_argument("--pullup-settle", type=float, default=0.5,
                        help="settle time after SOFT_CONN for a possible UDC re-init")
    tuning.add_argument("--bus-settle", type=float, default=1.0,
                        help="delay after HOST_CONNECT before the first SETUP")
    tuning.add_argument("--descriptor-settle", type=float, default=0.25,
                        help="delay between SET_ADDRESS and GET_DESCRIPTOR(config)")
    tuning.add_argument("--qmu-timeout", type=float, default=5.0,
                        help="seconds to wait for LOG QMU_READY after SET_CONFIGURATION")
    tuning.add_argument("--timeout", type=float, default=10.0,
                        help="per-command fastboot reply timeout")
    tuning.add_argument("--download-timeout", type=float, default=120.0,
                        help="timeout for the reply after download payload")
    tuning.add_argument("--flash-timeout", type=float, default=180.0,
                        help="timeout for the flash:<partition> reply")
    tuning.add_argument("--chunk-size", type=int, default=vusb.EP1_CHUNK_SIZE,
                        help="EP1 bulk OUT chunk size (default 16384; do not exceed)")
    tuning.add_argument("--chunk-delay", type=float, default=0.002,
                        help="seconds to sleep between EP1 OUT chunks when --pacing "
                             "delay (default 0.002: conservative; raise to 0.005/0.01 "
                             "if the download reply times out)")
    tuning.add_argument("--pacing", choices=["delay", "ack"], default="delay",
                        help="download flow control: 'delay' = fixed chunk sleep "
                             "(works on the current model), 'ack' = credit paced by "
                             "the model's per-RX-GPD ACK frames (0x83)")
    tuning.add_argument("--ack-window", type=int, default=4,
                        help="max chunks in flight with --pacing ack (default 4)")
    tuning.add_argument("--ack-timeout", type=float, default=10.0,
                        help="seconds to wait for one ACK with --pacing ack")
    tuning.add_argument("--ack-frame-type", type=lambda text: int(text, 0),
                        default=0x83, metavar="TYPE",
                        help="chardev frame type of the model RX-GPD ACK "
                             "(default 0x83)")

    output = parser.add_argument_group("output")
    output.add_argument("-v", "--verbose", action="count", default=0,
                        help="increase log verbosity (-v frame level, -vv hexdump level)")
    output.add_argument("-q", "--quiet", action="store_true",
                        help="no progress lines")
    output.add_argument("--log", metavar="FILE", default=None,
                        help="write a logging-module protocol log to FILE")
    output.add_argument("--dump-log", metavar="FILE", default=None,
                        help="write the full frame/event/LOG traffic dump to FILE "
                             "(useful for post-mortem download diagnostics)")

    actions = parser.add_argument_group("actions (applied in order)")
    actions.add_argument("--getvar", action="append", default=[], metavar="NAME",
                         help="read a fastboot variable (repeatable)")
    actions.add_argument("--flash", action="append", default=[], metavar="PART=PATH",
                         help="flash a local image file to PART (repeatable)")

    sub = parser.add_subparsers(dest="command", metavar="COMMAND")
    p_getvar = sub.add_parser("getvar", help="read fastboot variable(s)")
    p_getvar.add_argument("names", nargs="+", help="variable name(s), e.g. version product")
    p_download = sub.add_parser("download", help="raw download of a file")
    p_download.add_argument("path", help="local image file")
    p_flash = sub.add_parser("flash", help="flash an image file to a partition")
    p_flash.add_argument("partition", help="partition name, e.g. boot")
    p_flash.add_argument("path", help="local image file")
    p_reboot = sub.add_parser("reboot", help="reboot the device")
    p_reboot.add_argument("mode", nargs="?", default="normal",
                          choices=["normal", "bootloader", "fastboot", "recovery"],
                          help="reboot target (default normal)")
    p_oem = sub.add_parser("oem", help="pass an oem command through")
    p_oem.add_argument("args", nargs="+", help="arguments after 'oem '")
    p_cmd = sub.add_parser("command", help="send one raw fastboot command")
    p_cmd.add_argument("args", nargs="+", help="command words (joined with spaces)")
    return parser


def configure_logging(args) -> None:
    level = logging.WARNING
    if args.verbose == 1:
        level = logging.INFO
    elif args.verbose >= 2:
        level = logging.DEBUG
    handlers = []
    stream = logging.StreamHandler(sys.stderr)
    stream.setFormatter(logging.Formatter("[%(name)s] %(message)s"))
    handlers.append(stream)
    if args.log:
        file_handler = logging.FileHandler(args.log, encoding="utf-8")
        file_handler.setFormatter(
            logging.Formatter("%(asctime)s %(levelname)s [%(name)s] %(message)s")
        )
        handlers.append(file_handler)
    logging.basicConfig(level=level, handlers=handlers, force=True)


def make_client(args) -> fastboot.FastbootClient:
    return fastboot.FastbootClient(
        args.host,
        args.port,
        connect_timeout=args.connect_timeout,
        soft_conn_timeout=args.soft_conn_timeout,
        pullup_settle=args.pullup_settle,
        bus_settle=args.bus_settle,
        descriptor_settle=args.descriptor_settle,
        qmu_timeout=args.qmu_timeout,
        command_timeout=args.timeout,
        download_timeout=args.download_timeout,
        flash_timeout=args.flash_timeout,
        chunk_size=args.chunk_size,
        chunk_delay=args.chunk_delay,
        pacing=args.pacing,
        ack_window=args.ack_window,
        ack_timeout=args.ack_timeout,
        ack_frame_type=args.ack_frame_type,
        record_traffic=bool(args.dump_log),
    )


def flash_one(client: fastboot.FastbootClient, args, partition: str, path: str) -> None:
    data = fastboot.read_image_file(path)
    print("[flash] %s: %s (%s)" % (partition, path, human_bytes(len(data))))
    limit = client.ensure_downloadable(len(data))
    if limit is not None:
        sys.stderr.write("[flash] device max-download-size: %s\n"
                         % human_bytes(limit))
        sys.stderr.flush()
    progress = ProgressPrinter("download", len(data), quiet=args.quiet)
    started = time.monotonic()
    stats = client.download(data, progress=progress)
    download_seconds = time.monotonic() - started
    sys.stderr.write(
        "[download] %s in %.2fs (%s)\n"
        % (human_bytes(len(data)), download_seconds,
           human_rate(len(data), download_seconds))
    )
    sys.stderr.flush()
    started = time.monotonic()
    client.command("flash:%s" % partition, timeout=args.flash_timeout)
    flash_seconds = time.monotonic() - started
    sys.stderr.write("[flash] %s OKAY in %.2fs\n" % (partition, flash_seconds))
    sys.stderr.flush()
    print("[flash] %s: OKAY (download %s, acks=%d, write %.2fs)"
          % (partition, stats, stats.acks, flash_seconds))


def _require_image(path: str) -> str:
    image_path = os.path.expanduser(path)
    if not os.path.isfile(image_path):
        raise ValueError("image file not found: %s" % path)
    return image_path


def print_diagnostics(details) -> None:
    if details is None:
        return
    sys.stderr.write("[diag] failure snapshot:\n")
    for line in details.describe():
        sys.stderr.write("[diag]   %s\n" % line)
    sys.stderr.flush()


def run(args) -> int:
    if not args.getvar and not args.flash and args.command is None:
        build_parser().print_help(sys.stderr)
        return 2

    # Validate file-based actions before touching the device, so a typo does
    # not disturb a QEMU/LK session that is already running.
    flash_jobs = []
    for spec in args.flash:
        partition, path = parse_flash_spec(spec)
        flash_jobs.append((partition, _require_image(path)))
    if args.command == "flash":
        _require_image(args.path)
    elif args.command == "download":
        _require_image(args.path)

    client = make_client(args)
    started = time.monotonic()
    try:
        device = client.connect()
        serial = device.strings.get(device.device.iSerialNumber, "")
        sys.stderr.write(
            "[fastboot] device %04x:%04x%s ready\n"
            % (device.vendor_id, device.product_id,
               (" (" + serial + ")") if serial else "")
        )
        sys.stderr.flush()

        for name in args.getvar:
            print("%s: %s" % (name, client.getvar(name)))

        for partition, path in flash_jobs:
            flash_one(client, args, partition, path)

        command = args.command
        if command == "getvar":
            for name in args.names:
                print("%s: %s" % (name, client.getvar(name)))
        elif command == "download":
            data = fastboot.read_image_file(args.path)
            limit = client.ensure_downloadable(len(data))
            if limit is not None:
                sys.stderr.write("[download] device max-download-size: %s\n"
                                 % human_bytes(limit))
            progress = ProgressPrinter("download", len(data), quiet=args.quiet)
            started_dl = time.monotonic()
            stats = client.download(data, progress=progress)
            seconds = time.monotonic() - started_dl
            print("[download] %s in %.2fs (%s)"
                  % (human_bytes(len(data)), seconds, human_rate(len(data), seconds)))
            print("[download] OKAY (%d chunks, %d acks, pacing=%s)"
                  % (stats.chunk_count, stats.acks, args.pacing))
        elif command == "flash":
            flash_one(client, args, args.partition, args.path)
        elif command == "reboot":
            response = client.reboot(args.mode)
            print("[reboot] %s: %s"
                  % (args.mode, "OKAY" if response is not None else "sent (no reply)"))
        elif command == "oem":
            response = client.oem(*args.args)
            print("[oem] %s" % (response.text or response.status))
        elif command == "command":
            response = client.command(" ".join(args.args))
            print(response.text or response.status)

        total = time.monotonic() - started
        print("Finished. total time: %.3fs" % total)
        return 0
    except fastboot.FastbootFail as exc:
        sys.stderr.write("[fastboot] FAIL: %s\n" % exc.message)
        print_diagnostics(getattr(exc, "details", None))
        return 1
    except fastboot.FastbootDownloadError as exc:
        sys.stderr.write("[fastboot] download error: %s\n" % exc)
        print_diagnostics(exc.details)
        return 1
    except vusb.VusbTimeout as exc:
        sys.stderr.write("[fastboot] timeout: %s\n" % exc)
        if "ACK" in str(exc):
            sys.stderr.write(
                "[fastboot] hint: --pacing ack needs the model ACK frame (0x83); "
                "if the current model does not send it use --pacing delay, "
                "otherwise raise --ack-timeout / retry\n")
        return 1
    except (vusb.VusbError, fastboot.FastbootError) as exc:
        sys.stderr.write("[fastboot] error: %s\n" % exc)
        print_diagnostics(getattr(exc, "details", None))
        return 1
    except KeyboardInterrupt:
        sys.stderr.write("[fastboot] interrupted\n")
        return 130
    finally:
        if args.dump_log:
            try:
                path = client.usb.dump_traffic(args.dump_log)
                sys.stderr.write("[diag] traffic log written to %s\n" % path)
            except OSError as exc:
                sys.stderr.write("[diag] cannot write traffic log %s: %s\n"
                                 % (args.dump_log, exc))
        client.close()


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    configure_logging(args)
    try:
        return run(args)
    except ValueError as exc:
        sys.stderr.write("error: %s\n" % exc)
        return 2


if __name__ == "__main__":
    sys.exit(main())

