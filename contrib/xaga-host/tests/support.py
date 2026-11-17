"""Shared helpers for the host-side unit tests (stdlib unittest only)."""
from __future__ import annotations

import os
import subprocess
import sys
import unittest

HOST_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if HOST_DIR not in sys.path:
    sys.path.insert(0, HOST_DIR)

from lib import fastboot, mock_device, vusb  # noqa: E402

CLI_PATH = os.path.join(HOST_DIR, "fastboot_cli.py")

#: Fast bring-up timings for the mock device (the real model uses bigger ones).
FAST_CLIENT_KW = dict(
    connect_timeout=5.0,
    io_timeout=5.0,
    soft_conn_timeout=5.0,
    pullup_settle=0.02,
    bus_settle=0.02,
    descriptor_settle=0.005,
    qmu_timeout=2.0,
    command_timeout=3.0,
    download_timeout=10.0,
    flash_timeout=10.0,
)

#: Extra CLI flags matching FAST_CLIENT_KW.
FAST_CLI_FLAGS = [
    "--soft-conn-timeout", "5",
    "--pullup-settle", "0.02",
    "--bus-settle", "0.02",
    "--descriptor-settle", "0.005",
    "--qmu-timeout", "2",
    "--timeout", "3",
    "--download-timeout", "10",
    "--flash-timeout", "10",
    "-q",
]


class RecordingDevice(mock_device.MockDevice):
    """Mock device that only records EP1 OUT frames (for transport tests)."""

    def __init__(self, server):
        self.chunks = []
        super().__init__(server)

    def _reset_state(self) -> None:
        self.chunks = []

    def on_bulk_out(self, payload: bytes) -> None:
        self.chunks.append(bytes(payload))


class MockFastbootCase(unittest.TestCase):
    """Base class: ephemeral mock fastboot UDC + connected FastbootClient."""

    require_soft_conn = True
    split_responses = False

    def setUp(self) -> None:
        self.server = mock_device.MockChardevServer(mock_device.MockFastbootDevice).start()
        self.device_mock = self.server.device
        self.device_mock.require_soft_conn = self.require_soft_conn
        self.device_mock.split_responses = self.split_responses
        self.addCleanup(self.server.stop)
        self.client = fastboot.FastbootClient("127.0.0.1", self.server.port, **FAST_CLIENT_KW)
        self.addCleanup(self.client.close)

    def connect(self) -> "vusb.UsbDeviceInfo":
        return self.client.connect()


def run_cli(args, timeout: float = 60.0):
    """Run fastboot_cli.py in a subprocess, returning CompletedProcess."""
    command = [sys.executable, CLI_PATH] + list(args)
    return subprocess.run(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout,
        text=True,
    )
