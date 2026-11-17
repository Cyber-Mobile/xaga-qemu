#!/usr/bin/env python3
"""Download/flash throughput benchmark against the in-process mock UDC.

This does not measure QEMU speed, only the host-side Python cost (frame
synthesis, chunk loop, progress) for a 64 MiB image, and how much pacing
(--chunk-delay) costs.

Usage:
    python3 tests/bench_mock.py [SIZE_MIB] [CHUNK_DELAY_SECONDS | ack]
"""
import os
import sys
import time

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
HOST_DIR = os.path.dirname(TESTS_DIR)
for _path in (HOST_DIR, TESTS_DIR):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from lib import fastboot, mock_device  # noqa: E402

MIB = 1024.0 * 1024.0


def run_once(image, chunk_delay, ack=False, ack_window=4):
    server = mock_device.MockChardevServer(mock_device.MockFastbootDevice).start()
    server.device.ack_frames = ack
    client = fastboot.FastbootClient(
        "127.0.0.1",
        server.port,
        soft_conn_timeout=5.0,
        pullup_settle=0.02,
        bus_settle=0.02,
        descriptor_settle=0.005,
        qmu_timeout=2.0,
        command_timeout=5.0,
        download_timeout=30.0,
        flash_timeout=30.0,
        chunk_delay=chunk_delay,
        pacing="ack" if ack else "delay",
        ack_window=ack_window,
    )
    try:
        client.connect()
        started = time.monotonic()
        stats = client.download(image, chunk_delay=chunk_delay)
        download_seconds = time.monotonic() - started
        started = time.monotonic()
        client.command("flash:boot", timeout=30.0)
        flash_seconds = time.monotonic() - started
        received = server.device.downloads[-1]
        assert received == image, "mock device received corrupted image"
        return stats, download_seconds, flash_seconds
    finally:
        client.close()
        server.stop()


def main(argv):
    size_mib = int(argv[1]) if len(argv) > 1 else 64
    mode = argv[2] if len(argv) > 2 else ""
    image = os.urandom(size_mib * 1024 * 1024)
    print("image: %d MiB, %d chunks of 16 KiB"
          % (size_mib, (len(image) + 16383) // 16384))
    if mode == "ack":
        for window in (1, 4, 16):
            started = time.monotonic()
            stats, download_seconds, flash_seconds = run_once(
                image, 0.0, ack=True, ack_window=window)
            total = time.monotonic() - started
            print("pacing=ack window=%-2d  download %6.2fs (%6.1f MiB/s)  "
                  "acks=%d  flash-cmd %.3fs  end-to-end %.2fs"
                  % (window, download_seconds,
                     len(image) / download_seconds / MIB, stats.acks,
                     flash_seconds, total))
        return 0
    delays = [float(mode)] if mode else [0.0, 0.002]
    for delay in delays:
        started = time.monotonic()
        stats, download_seconds, flash_seconds = run_once(image, delay)
        total = time.monotonic() - started
        print("chunk_delay=%.3fs  download %6.2fs (%6.1f MiB/s)  flash-cmd %.3fs  "
              "end-to-end %.2fs"
              % (delay, download_seconds, len(image) / download_seconds / MIB,
                 flash_seconds, total))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
