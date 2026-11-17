"""Transport, control-transfer and enumeration tests (mock UDC, no QEMU)."""
import os
import socket
import sys
import time
import unittest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
HOST_DIR = os.path.dirname(TESTS_DIR)
for _path in (HOST_DIR, TESTS_DIR):
    if _path not in sys.path:
        sys.path.insert(0, _path)

import support  # noqa: E402
from lib import mock_device, vusb  # noqa: E402


class EnumerationTests(unittest.TestCase):
    def setUp(self):
        self.server = mock_device.MockChardevServer(mock_device.MockFastbootDevice).start()
        self.addCleanup(self.server.stop)
        self.usb = vusb.VUsbHost(
            "127.0.0.1", self.server.port, connect_timeout=5.0, io_timeout=2.0
        )
        self.addCleanup(self.usb.close)

    def bring_up(self, descriptor_settle=0.005):
        self.usb.connect()
        self.assertTrue(self.usb.wait_ready(2.0))
        self.usb.wait_pullup(timeout=3.0, settle=0.02)
        self.assertTrue(self.usb.connect_bus())
        self.usb.pump(0.02)
        return self.usb.enumerate(descriptor_settle=descriptor_settle)

    def test_enumerate_standard_sequence(self):
        info = self.bring_up()
        self.assertEqual(info.device.idVendor, 0x18D1)
        self.assertEqual(info.device.idProduct, 0xD00D)
        self.assertIsNotNone(info.config)
        self.assertEqual(info.config.bConfigurationValue, 1)
        self.assertEqual([ep.bEndpointAddress for ep in info.config.endpoints], [0x81, 0x01])
        self.assertEqual(info.bulk_in.bEndpointAddress, 0x81)
        self.assertEqual(info.bulk_out.bEndpointAddress, 0x01)
        self.assertEqual(info.device.bNumConfigurations, 1)
        self.assertIn("18d1:d00d", info.summary())

        mock = self.server.device
        self.assertTrue(self.usb.wait_log("QMU_READY", 1.0))
        self.assertEqual(mock.address, 1)
        self.assertTrue(mock.configured)
        requests = [setup[1] for setup in mock.setups]
        self.assertIn(vusb.USB_REQ_GET_DESCRIPTOR, requests)
        self.assertIn(vusb.USB_REQ_SET_ADDRESS, requests)
        self.assertIn(vusb.USB_REQ_SET_CONFIGURATION, requests)
        self.assertEqual(self.usb.log_count("SOFT_CONN"), 1)

    def test_get_string_descriptor(self):
        self.bring_up()
        self.assertEqual(self.usb.get_string(2), "xaga")
        self.assertEqual(self.usb.get_string(1), "Xiaomi")

    def test_enumerate_with_strings(self):
        self.usb.connect()
        self.usb.wait_ready(2.0)
        self.usb.wait_pullup(timeout=3.0, settle=0.02)
        self.usb.connect_bus()
        self.usb.pump(0.02)
        info = self.usb.enumerate(include_strings=True, descriptor_settle=0.005)
        self.assertEqual(info.strings.get(2), "xaga")

    def test_control_in_timeout_for_unknown_descriptor(self):
        self.bring_up()
        started = time.monotonic()
        with self.assertRaises(vusb.VusbTimeout):
            self.usb.get_descriptor(0xEE, length=8, timeout=0.3)
        self.assertLess(time.monotonic() - started, 2.0)

    def test_wait_pullup_timeout(self):
        server = mock_device.MockChardevServer(mock_device.MockFastbootDevice).start()
        self.addCleanup(server.stop)
        server.device.soft_conn_delay = 5.0
        usb = vusb.VUsbHost("127.0.0.1", server.port, connect_timeout=5.0, io_timeout=1.0)
        self.addCleanup(usb.close)
        usb.connect()
        self.assertTrue(usb.wait_ready(2.0))
        with self.assertRaises(vusb.VusbTimeout):
            usb.wait_pullup(timeout=0.3, settle=0.02)

    def test_not_ready_drops_setup_before_pullup(self):
        server = mock_device.MockChardevServer(mock_device.MockFastbootDevice).start()
        self.addCleanup(server.stop)
        server.device.soft_conn_delay = 0.3
        usb = vusb.VUsbHost("127.0.0.1", server.port, connect_timeout=5.0, io_timeout=1.0)
        self.addCleanup(usb.close)
        usb.connect()
        self.assertTrue(usb.wait_ready(2.0))
        # Deliberately send a control request while LK has not raised D+ yet.
        usb._send_setup(vusb.USB_DIR_IN, vusb.USB_REQ_GET_DESCRIPTOR, 0x0100, 0, 64)
        self.assertTrue(usb.wait_log("NOT_READY", 2.0))
        usb.wait_pullup(timeout=3.0, settle=0.02)
        self.assertTrue(usb.connect_bus())
        usb.pump(0.02)
        info = usb.enumerate(descriptor_settle=0.005)
        self.assertEqual(info.device.idVendor, 0x18D1)

    def test_connect_refused(self):
        probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
        probe.close()
        usb = vusb.VUsbHost("127.0.0.1", port, connect_timeout=0.4)
        with self.assertRaises(vusb.VusbConnectionError):
            usb.connect()

    def test_reconnect(self):
        self.bring_up()
        self.usb.close()
        self.assertFalse(self.usb.connected)
        self.usb.connect()
        self.assertTrue(self.usb.wait_ready(2.0))
        self.usb.wait_pullup(timeout=3.0, settle=0.02)
        self.usb.connect_bus()
        self.usb.pump(0.02)
        info = self.usb.enumerate(descriptor_settle=0.005)
        self.assertEqual(info.device.idProduct, 0xD00D)

    def test_bulk_out_chunking(self):
        server = mock_device.MockChardevServer(support.RecordingDevice).start()
        self.addCleanup(server.stop)
        usb = vusb.VUsbHost("127.0.0.1", server.port, connect_timeout=5.0, io_timeout=2.0)
        self.addCleanup(usb.close)
        usb.connect()
        self.assertTrue(usb.wait_ready(2.0))
        usb.wait_pullup(timeout=3.0, settle=0.02)
        payload = bytes((i * 37 + 11) & 0xFF for i in range(40000))
        stats = usb.bulk_out(payload)
        self.assertEqual(stats.byte_count, 40000)
        self.assertEqual(stats.chunk_count, 3)
        self.assertGreater(stats.bytes_per_second, 0)
        self.assertTrue(
            mock_device.wait_until(lambda: len(server.device.chunks) == 3, 2.0)
        )
        chunks = server.device.chunks
        self.assertEqual([len(chunk) for chunk in chunks], [16384, 16384, 7232])
        self.assertEqual(b"".join(chunks), payload)

    def test_bulk_out_invalid_chunk_size(self):
        with self.assertRaises(ValueError):
            self.usb.bulk_out(b"x", chunk_size=0)
        with self.assertRaises(ValueError):
            self.usb.bulk_out(b"x", chunk_size=vusb.MAX_FRAME_PAYLOAD + 1)

    def test_bulk_out_raw_single_frame(self):
        server = mock_device.MockChardevServer(support.RecordingDevice).start()
        self.addCleanup(server.stop)
        usb = vusb.VUsbHost("127.0.0.1", server.port, connect_timeout=5.0, io_timeout=2.0)
        self.addCleanup(usb.close)
        usb.connect()
        self.assertTrue(usb.wait_ready(2.0))
        usb.wait_pullup(timeout=3.0, settle=0.02)
        usb.bulk_out_raw(b"getvar:version")
        self.assertTrue(
            mock_device.wait_until(lambda: server.device.chunks == [b"getvar:version"], 2.0)
        )


if __name__ == "__main__":
    unittest.main()
