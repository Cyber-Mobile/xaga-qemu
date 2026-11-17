"""End-to-end fastboot protocol tests against the mock LK device."""
import os
import sys
import tempfile
import time
import unittest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
HOST_DIR = os.path.dirname(TESTS_DIR)
for _path in (HOST_DIR, TESTS_DIR):
    if _path not in sys.path:
        sys.path.insert(0, _path)

import support  # noqa: E402
from lib import fastboot, mock_device, vusb  # noqa: E402


class FastbootBringupTests(support.MockFastbootCase):
    def test_connect_and_enumeration(self):
        info = self.connect()
        self.assertEqual(info.device.idVendor, 0x18D1)
        self.assertEqual(info.device.idProduct, 0xD00D)
        self.assertTrue(self.client.connected)
        self.assertTrue(self.server.device.configured)

    def test_getvar_version_product_slot(self):
        self.connect()
        self.assertEqual(self.client.getvar("version"), "0.5")
        self.assertEqual(self.client.getvar("product"), "xaga")
        self.assertEqual(self.client.getvar("current-slot"), "a")
        self.assertEqual(self.client.getvar("serialno"), "0xdeadbeef")

    def test_getvar_unknown_raises_fastboot_fail(self):
        self.connect()
        with self.assertRaises(fastboot.FastbootFail) as ctx:
            self.client.getvar("no-such-variable")
        self.assertIn("GetVar Variable not found", ctx.exception.message)

    def test_raw_command_unknown_raises(self):
        self.connect()
        with self.assertRaises(fastboot.FastbootFail):
            self.client.command("definitely-not-a-command")

    def test_erase_okay(self):
        self.connect()
        response = self.client.erase("userdata")
        self.assertEqual(response.status, "OKAY")

    def test_oem_passthrough(self):
        self.connect()
        response = self.client.oem("device-info")
        self.assertEqual(response.status, "OKAY")
        self.assertIn(b"oem device-info", self.server.device.commands)


class FastbootDownloadFlashTests(support.MockFastbootCase):
    @staticmethod
    def make_image(size):
        return bytes((i * 31 + 7) & 0xFF for i in range(size))

    def test_download_exact_bytes(self):
        self.connect()
        image = self.make_image(40000)  # 3 EP1 chunks: 16384 + 16384 + 7232
        stats = self.client.download(image)
        self.assertEqual(stats.byte_count, 40000)
        self.assertEqual(stats.chunk_count, 3)
        self.assertTrue(self.device_mock.wait_for(lambda: len(self.device_mock.downloads) == 1, 2))
        self.assertEqual(self.device_mock.downloads[0], image)

    def test_download_progress_callback(self):
        self.connect()
        image = self.make_image(50000)
        calls = []
        self.client.download(image, progress=lambda done, total, elapsed: calls.append((done, total)))
        self.assertGreaterEqual(len(calls), 4)  # ceil(50000/16384) chunks
        self.assertEqual(calls[0][1], 50000)
        self.assertEqual(calls[-1][0], 50000)
        self.assertEqual(calls[-1][1], 50000)

    def test_download_large_image(self):
        self.connect()
        image = os.urandom(2 * 1024 * 1024 + 123)
        stats = self.client.download(image)
        self.assertEqual(stats.byte_count, len(image))
        self.assertEqual(stats.chunk_count, (len(image) + 16383) // 16384)
        self.assertTrue(self.device_mock.wait_for(lambda: len(self.device_mock.downloads) == 1, 3))
        self.assertEqual(self.device_mock.downloads[0], image)

    def test_flash_image_bytes(self):
        self.connect()
        image = self.make_image(20000)
        response = self.client.flash("boot", image)
        self.assertEqual(response.status, "OKAY")
        self.assertTrue(self.device_mock.wait_for(lambda: len(self.device_mock.flashes) == 1, 2))
        self.assertEqual(self.device_mock.flashes[0], ("boot", image))

    def test_flash_image_file(self):
        self.connect()
        image = self.make_image(7000)
        handle = tempfile.NamedTemporaryFile(delete=False)
        try:
            handle.write(image)
            handle.close()
            self.client.flash("vendor_boot", path=handle.name)
        finally:
            os.unlink(handle.name)
        self.assertTrue(self.device_mock.wait_for(lambda: len(self.device_mock.flashes) == 1, 2))
        self.assertEqual(self.device_mock.flashes[0], ("vendor_boot", image))

    def test_flash_unknown_partition_fails(self):
        self.connect()
        with self.assertRaises(fastboot.FastbootFail) as ctx:
            self.client.flash("not_a_partition", self.make_image(1024))
        self.assertIn("partition not found", ctx.exception.message)

    def test_flash_write_failure_fails(self):
        self.device_mock.fail_partitions.add("boot")
        self.connect()
        with self.assertRaises(fastboot.FastbootFail) as ctx:
            self.client.flash("boot", self.make_image(1024))
        self.assertIn("write failure", ctx.exception.message)

    def test_flash_requires_data_or_path(self):
        self.connect()
        with self.assertRaises(ValueError):
            self.client.flash("boot")

    def test_reboot_reports_okay_and_closes(self):
        self.connect()
        response = self.client.reboot("normal")
        self.assertIsNotNone(response)
        self.assertEqual(response.status, "OKAY")
        self.assertTrue(
            mock_device.wait_until(lambda: self.device_mock.closed, 2.0)
        )

    def test_reboot_bootloader_command_name(self):
        self.connect()
        self.client.reboot("bootloader")
        self.assertTrue(
            self.device_mock.wait_for(lambda: b"reboot-bootloader" in self.device_mock.commands, 2)
        )

    def test_reboot_unknown_mode_raises(self):
        with self.assertRaises(ValueError):
            self.client.reboot("banana")


class SplitResponseTests(support.MockFastbootCase):
    """The model may split one fastboot reply over several 0x82 frames."""

    split_responses = True

    def test_getvar_reassembly(self):
        self.connect()
        self.assertEqual(self.client.getvar("version"), "0.5")
        self.assertEqual(self.client.getvar("product"), "xaga")

    def test_download_and_flash_with_split_replies(self):
        self.connect()
        image = bytes(range(200))
        self.client.flash("dtbo", image)
        self.assertTrue(self.device_mock.wait_for(lambda: len(self.device_mock.flashes) == 1, 2))
        self.assertEqual(self.device_mock.flashes[0], ("dtbo", image))


class DownloadDiagnosticsTests(support.MockFastbootCase):
    def test_device_fail_carries_diagnostics(self):
        self.device_mock.fail_download = True
        self.device_mock.download_fail_info = "usb read failed"
        self.connect()
        image = bytes(range(256)) * 200  # 51200 bytes -> 4 chunks
        with self.assertRaises(fastboot.FastbootFail) as ctx:
            self.client.download(image)
        exc = ctx.exception
        self.assertIn("Read USB error", exc.message)
        details = exc.details
        self.assertIsNotNone(details)
        self.assertEqual(details.stage, "device-FAIL")
        self.assertEqual(details.sent_bytes, len(image))
        self.assertEqual(details.chunks_sent, 4)
        self.assertEqual(details.info_lines, ["usb read failed"])
        self.assertIn("acks=", "\n".join(details.describe()))

    def test_device_fail_without_info_line(self):
        self.device_mock.fail_download = True
        self.connect()
        with self.assertRaises(fastboot.FastbootFail) as ctx:
            self.client.download(b"z" * 100)
        self.assertEqual(ctx.exception.details.info_lines, [])
        self.assertEqual(ctx.exception.details.pacing, "delay")


class MaxDownloadSizeTests(support.MockFastbootCase):
    def test_getvar_hex_parse(self):
        self.connect()
        self.assertEqual(self.client.get_max_download_size(), 0x04000000)

    def test_getvar_plain_hex_parse(self):
        self.device_mock.vars["max-download-size"] = "01000000"
        self.connect()
        self.assertEqual(self.client.get_max_download_size(), 0x01000000)

    def test_unknown_max_download_size_allows_flash(self):
        self.device_mock.vars.pop("max-download-size")
        self.connect()
        self.assertIsNone(self.client.get_max_download_size())
        self.client.flash("boot", b"\x00" * 4096)
        self.assertTrue(self.device_mock.wait_for(
            lambda: len(self.device_mock.flashes) == 1, 2))

    def test_flash_rejects_oversize_image_before_download(self):
        self.device_mock.vars["max-download-size"] = "0x00100000"  # 1 MiB
        self.connect()
        with self.assertRaises(fastboot.FastbootError) as ctx:
            self.client.flash("boot", b"\x00" * (2 * 1024 * 1024))
        self.assertNotIsInstance(ctx.exception, fastboot.FastbootFail)
        self.assertIn("max-download-size", str(ctx.exception))
        self.assertEqual(self.device_mock.downloads, [])

    def test_flash_allows_exact_limit(self):
        self.device_mock.vars["max-download-size"] = "0x00040000"  # 256 KiB
        self.connect()
        self.client.flash("boot", b"\x00" * (256 * 1024))
        self.assertTrue(self.device_mock.wait_for(
            lambda: len(self.device_mock.flashes) == 1, 2))


class LateConnectTests(support.MockFastbootCase):
    """Host connects after LK already pulled D+ up: model does not replay SOFT_CONN."""

    def test_enumeration_without_soft_conn_log(self):
        self.device_mock.log_soft_conn = False
        self.client.soft_conn_timeout = 0.2
        info = self.client.connect()
        self.assertEqual(info.device.idVendor, 0x18D1)
        self.assertEqual(self.client.getvar("version"), "0.5")
        self.assertEqual(self.client.getvar("product"), "xaga")


@unittest.skipUnless(
    os.environ.get("VUSB_BIG_TEST"),
    "set VUSB_BIG_TEST=1 to run the 64 MiB download regression",
)
class BigImageTests(support.MockFastbootCase):
    def test_64mb_download_and_flash(self):
        self.connect()
        size = 64 * 1024 * 1024
        image = os.urandom(size)
        started = time.monotonic()
        self.client.flash("boot_a", image)
        elapsed = time.monotonic() - started
        self.assertTrue(self.device_mock.wait_for(lambda: len(self.device_mock.flashes) == 1, 10))
        self.assertEqual(self.device_mock.flashes[0][0], "boot_a")
        self.assertEqual(len(self.device_mock.flashes[0][1]), size)
        self.assertLess(elapsed, 120.0)


if __name__ == "__main__":
    unittest.main()
