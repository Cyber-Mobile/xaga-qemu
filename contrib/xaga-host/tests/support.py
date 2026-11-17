"""Shared helpers for the host-side unit tests (stdlib unittest only)."""
from __future__ import annotations

import os
import subprocess
import sys
import unittest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
HOST_DIR = os.path.dirname(TESTS_DIR)
if HOST_DIR not in sys.path:
    sys.path.insert(0, HOST_DIR)

from lib import adb, fastboot, mock_device, vusb  # noqa: E402

CLI_PATH = os.path.join(HOST_DIR, "fastboot_cli.py")
ADB_CLI_PATH = os.path.join(HOST_DIR, "adb_cli.py")
TEST_DATA_DIR = os.path.join(TESTS_DIR, "fixtures")
TEST_KEY_PATH = os.path.join(TEST_DATA_DIR, "adbkey.pem")
TEST_KEY_PKCS8_PATH = os.path.join(TEST_DATA_DIR, "adbkey_pkcs8.pem")
TEST_PUB_PATH = os.path.join(TEST_DATA_DIR, "adbkey.pub.pem")

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


#: Fast bring-up timings for the mock adb gadget.
FAST_ADB_KW = dict(
    connect_timeout=5.0,
    io_timeout=5.0,
    soft_conn_timeout=5.0,
    pullup_settle=0.02,
    bus_settle=0.02,
    qmu_timeout=2.0,
    response_idle=0.005,
)

#: Extra CLI flags matching FAST_ADB_KW.
FAST_ADB_CLI_FLAGS = [
    "--connect-timeout", "5",
    "--soft-conn-timeout", "5",
    "--pullup-settle", "0.02",
    "--bus-settle", "0.02",
    "--qmu-timeout", "2",
    "--timeout", "15",
    "-q",
]


class MockAdbCase(unittest.TestCase):
    """Base class: ephemeral mock adb gadget + AdbClient (not yet connected)."""

    def setUp(self) -> None:
        self.server = mock_device.MockChardevServer(mock_device.MockAdbDevice).start()
        self.device_mock = self.server.device
        self.addCleanup(self.server.stop)
        self.client = adb.AdbClient(
            host="127.0.0.1", port=self.server.port, **FAST_ADB_KW
        )
        self.addCleanup(self.client.close)

    def connect(self, *, handshake: bool = True):
        device = self.client.connect()
        if handshake:
            self.banner = self.client.handshake(timeout=5.0)
        return device

    def load_test_key(self) -> "adb.RsaKey":
        return adb.RsaKey.load(TEST_KEY_PATH)


def make_temp_package(size: int, seed: int = 1) -> Tuple[str, bytes]:
    """Create a temporary pseudo-random package file; returns (path, data)."""
    import tempfile

    data = bytes(((index * 131 + seed * 17) ^ (index >> 8)) & 0xFF
                 for index in range(size))
    handle = tempfile.NamedTemporaryFile(delete=False, suffix=".zip")
    handle.write(data)
    handle.close()
    return handle.name, data


def run_adb_cli(args, timeout: float = 120.0):
    """Run adb_cli.py in a subprocess, returning CompletedProcess."""
    command = [sys.executable, ADB_CLI_PATH] + list(args)
    return subprocess.run(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout,
        text=True,
    )

