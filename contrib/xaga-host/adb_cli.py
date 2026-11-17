#!/usr/bin/env python3
"""adb CLI for the MT6895 QEMU virtual USB gadget (USB function-fs transport).

Commands
--------
    devices                      connect + CNXN/AUTH, print "<serial>\t<state>"
    shell <command...>           run a shell command (shell protocol v2 when available)
    sideload <package.zip>       sideload an OTA package (minadbd sideload-host protocol)
    reboot [target]              reboot; target: default|bootloader|recovery|fastboot|rescue
    push <local> <remote>        adb push (sync SEND sub-protocol)

Examples
--------
    python3 adb_cli.py --host 127.0.0.1 --port 4444 devices
    python3 adb_cli.py --port 4444 shell getprop ro.product.device
    python3 adb_cli.py --port 4444 sideload /Users/furruka/lineage-20-xaga.zip
    python3 adb_cli.py --port 4444 --timeout 600 sideload lineage.zip --block-size 65536
    python3 adb_cli.py --port 4444 reboot recovery

The device is reached through the QEMU chardev socket (same as fastboot_cli.py).
Exit codes: 0 success, 1 protocol/transport failure, 2 usage error.
"""
from __future__ import annotations

import argparse
import logging
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from lib import adb, vusb  # noqa: E402

MIB = 1024.0 * 1024.0
GIB = 1024.0 * MIB
CLI_VERSION = "1.0.0"


def human_bytes(count: int) -> str:
    count = float(count)
    if count >= GIB:
        return "%.2f GiB" % (count / GIB)
    if count >= MIB:
        return "%.1f MiB" % (count / MIB)
    if count >= 1024.0:
        return "%.1f KiB" % (count / 1024.0)
    return "%d B" % int(count)


def human_rate(byte_count: int, seconds: float) -> str:
    if seconds <= 0:
        return "-"
    return "%.1f MiB/s" % (byte_count / seconds / MIB)


class SideloadProgressPrinter:
    """Print sideload progress on stderr (AOSP-style ~percent estimate)."""

    def __init__(self, path: str, *, quiet: bool = False, interval: float = 0.5) -> None:
        self.path = path
        self.quiet = quiet
        self.interval = interval
        self._last_percent = -1.0
        self._last_time = 0.0

    def __call__(self, stats: adb.SideloadStats) -> None:
        if self.quiet:
            return
        now = time.monotonic()
        percent = stats.percent
        if (
            percent < 100.0
            and percent - self._last_percent < 2.0
            and now - self._last_time < self.interval
        ):
            return
        self._last_percent = percent
        self._last_time = now
        sys.stderr.write(
            "\r[sideload] %-28s (~%3.0f%%) %s / %s  %s   "
            % (
                os.path.basename(self.path)[:28],
                percent,
                human_bytes(stats.bytes_served),
                human_bytes(stats.file_size),
                human_rate(stats.bytes_served, stats.seconds),
            )
        )
        sys.stderr.flush()

    def finish(self) -> None:
        if not self.quiet:
            sys.stderr.write("\n")
            sys.stderr.flush()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="adb_cli.py",
        description="adb client for the MT6895 QEMU virtual USB gadget.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--host", default="127.0.0.1",
                        help="QEMU chardev host (default 127.0.0.1)")
    parser.add_argument("--port", type=int, default=4444,
                        help="QEMU chardev TCP port (default 4444)")
    parser.add_argument("--version", action="version", version="%(prog)s " + CLI_VERSION)

    tuning = parser.add_argument_group("bring-up / protocol tuning")
    tuning.add_argument("--connect-timeout", type=float, default=30.0,
                        help="seconds to retry the chardev TCP connect")
    tuning.add_argument("--soft-conn-timeout", type=float, default=150.0,
                        help="seconds to wait for the U2 pull-up (LOG SOFT_CONN)")
    tuning.add_argument("--pullup-settle", type=float, default=0.5,
                        help="settle time after SOFT_CONN")
    tuning.add_argument("--bus-settle", type=float, default=1.0,
                        help="delay after HOST_CONNECT before the first SETUP")
    tuning.add_argument("--descriptor-settle", type=float, default=0.25,
                        help="delay between SET_ADDRESS and GET_DESCRIPTOR(config)")
    tuning.add_argument("--qmu-timeout", type=float, default=5.0,
                        help="seconds to wait for LOG QMU_READY")
    tuning.add_argument("--timeout", type=float, default=30.0,
                        help="per-operation timeout in seconds")
    tuning.add_argument("--key", metavar="FILE", default=None,
                        help="adb private key (default: $ADBKEY or ~/.android/adbkey; "
                             "auto-generated with openssl when missing)")
    tuning.add_argument("--maxdata", type=int, default=adb.A_MAXDATA,
                        help="adb transport max payload (default 4096)")

    output = parser.add_argument_group("output")
    output.add_argument("-v", "--verbose", action="count", default=0,
                        help="increase log verbosity")
    output.add_argument("-q", "--quiet", action="store_true",
                        help="no progress lines")
    output.add_argument("--log", metavar="FILE", default=None,
                        help="write a logging-module protocol log to FILE")

    sub = parser.add_subparsers(dest="command", metavar="COMMAND")

    sub.add_parser("devices", help="list the device (adb devices style)")

    p_shell = sub.add_parser("shell", help="run a shell command")
    p_shell.add_argument("args", nargs=argparse.REMAINDER,
                         help="command and arguments")
    p_shell.add_argument("--legacy", action="store_true",
                         help="force the legacy shell service (no shell_v2)")

    p_side = sub.add_parser("sideload", help="sideload an OTA package")
    p_side.add_argument("path", help="local OTA/zip package")
    p_side.add_argument("--block-size", type=int, default=adb.SIDELOAD_BLOCK_SIZE,
                        help="sideload-host block size (default 65536)")
    p_side.add_argument("--no-legacy-fallback", action="store_true",
                        help="do not fall back to the legacy sideload: service")

    p_reboot = sub.add_parser("reboot", help="reboot the device")
    p_reboot.add_argument("target", nargs="?", default="",
                          choices=["", "bootloader", "recovery", "fastboot", "rescue"],
                          help="reboot target (default: normal boot)")

    p_push = sub.add_parser("push", help="push a file (sync SEND)")
    p_push.add_argument("local", help="local file")
    p_push.add_argument("remote", help="remote path, e.g. /data/local/tmp/x")

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


def make_client(args) -> adb.AdbClient:
    return adb.AdbClient(
        host=args.host,
        port=args.port,
        maxdata=args.maxdata,
        connect_timeout=args.connect_timeout,
        soft_conn_timeout=args.soft_conn_timeout,
        pullup_settle=args.pullup_settle,
        bus_settle=args.bus_settle,
        qmu_timeout=args.qmu_timeout,
        io_timeout=max(args.timeout, 5.0),
        key_path=args.key,
    )


def connect(args) -> adb.AdbClient:
    client = make_client(args)
    try:
        device = client.connect()
        sys.stderr.write("[adb] gadget enumerated: %s\n" % device.summary())
        banner = client.handshake(timeout=args.timeout)
        sys.stderr.write(
            "[adb] connected: state=%s device=%s serial=%s auth=%s maxdata=%d "
            "features=%s\n"
            % (banner.state, banner.device_name, banner.serial,
               client.auth_used, client.maxdata, ",".join(sorted(banner.features)))
        )
        sys.stderr.flush()
        return client
    except BaseException:
        client.close()
        raise


def cmd_devices(args) -> int:
    client = connect(args)
    try:
        print("%s\t%s" % (client.device_label(), client.device_state()))
        return 0
    finally:
        client.close()


def cmd_shell(args) -> int:
    words = list(args.args)
    if words and words[0] == "--":
        words = words[1:]
    if not words:
        sys.stderr.write("error: shell needs a command\n")
        return 2
    command = " ".join(words)
    client = connect(args)
    try:
        result = client.shell(
            command, timeout=args.timeout,
            protocol="legacy" if args.legacy else "auto",
        )
        if result.stdout:
            sys.stdout.buffer.write(result.stdout)
            sys.stdout.buffer.flush()
        if result.stderr:
            sys.stderr.buffer.write(result.stderr)
            sys.stderr.buffer.flush()
        if args.verbose:
            sys.stderr.write("[adb] shell protocol=%s exit=%s\n"
                             % (result.protocol, result.exit_code))
        return result.exit_code if result.exit_code is not None else 0
    finally:
        client.close()


def cmd_sideload(args) -> int:
    path = os.path.expanduser(args.path)
    if not os.path.isfile(path):
        sys.stderr.write("error: package not found: %s\n" % args.path)
        return 2
    size = os.path.getsize(path)
    sys.stderr.write("[sideload] package %s (%s)\n" % (path, human_bytes(size)))
    sys.stderr.flush()
    client = connect(args)
    printer = SideloadProgressPrinter(path, quiet=args.quiet)
    try:
        stats = client.sideload(
            path,
            block_size=args.block_size,
            progress=printer,
            timeout=args.timeout,
            legacy_fallback=not args.no_legacy_fallback,
        )
        printer.finish()
        ratio = float(stats.bytes_served) / float(stats.file_size or 1)
        print("Total xfer: %.2fx" % ratio)
        print("[sideload] OKAY (protocol=%s, %d blocks, %s in %.2fs, %s)"
              % (stats.protocol, stats.blocks, human_bytes(stats.bytes_served),
                 stats.seconds, human_rate(stats.bytes_served, stats.seconds)))
        return 0
    finally:
        printer.finish()
        client.close()


def cmd_reboot(args) -> int:
    client = connect(args)
    try:
        service = client.reboot(args.target, timeout=args.timeout)
        print("[reboot] sent %r" % service)
        return 0
    finally:
        client.close()


def cmd_push(args) -> int:
    local = os.path.expanduser(args.local)
    if not os.path.isfile(local):
        sys.stderr.write("error: local file not found: %s\n" % args.local)
        return 2
    size = os.path.getsize(local)
    client = connect(args)
    last = [0.0]

    def progress(done: int, total: int) -> None:
        if args.quiet:
            return
        now = time.monotonic()
        if done < total and now - last[0] < 0.3:
            return
        last[0] = now
        sys.stderr.write("\r[push] %s / %s   " % (human_bytes(done), human_bytes(total)))
        sys.stderr.flush()

    try:
        client.push(local, args.remote, timeout=max(args.timeout, 120.0),
                    progress=progress)
        if not args.quiet:
            sys.stderr.write("\n")
        print("[push] %s -> %s (%s)" % (args.local, args.remote, human_bytes(size)))
        return 0
    finally:
        client.close()


def run(args) -> int:
    dispatch = {
        "devices": cmd_devices,
        "shell": cmd_shell,
        "sideload": cmd_sideload,
        "reboot": cmd_reboot,
        "push": cmd_push,
    }
    handler = dispatch.get(args.command)
    if handler is None:
        build_parser().print_help(sys.stderr)
        return 2
    return handler(args)


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    configure_logging(args)
    try:
        return run(args)
    except adb.AdbAuthError as exc:
        sys.stderr.write(
            "[adb] authentication failed: %s\n"
            "[adb] hint: the device must authorize this host key; recovery adbd "
            "usually runs with authentication disabled\n" % exc
        )
        return 1
    except adb.SideloadError as exc:
        sys.stderr.write("[adb] sideload failed: %s\n" % exc)
        return 1
    except adb.AdbError as exc:
        sys.stderr.write("[adb] error: %s\n" % exc)
        return 1
    except vusb.VusbError as exc:
        sys.stderr.write("[adb] transport error: %s\n" % exc)
        return 1
    except KeyboardInterrupt:
        sys.stderr.write("[adb] interrupted\n")
        return 130
    except ValueError as exc:
        sys.stderr.write("error: %s\n" % exc)
        return 2


if __name__ == "__main__":
    sys.exit(main())
