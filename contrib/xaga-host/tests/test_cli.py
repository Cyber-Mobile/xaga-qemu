"""fastboot_cli.py subprocess tests against the mock LK device."""
import os
import sys
import tempfile
import unittest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
HOST_DIR = os.path.dirname(TESTS_DIR)
for _path in (HOST_DIR, TESTS_DIR):
    if _path not in sys.path:
        sys.path.insert(0, _path)

import support  # noqa: E402


class CliTests(support.MockFastbootCase):
    def cli(self, *extra):
        args = ["--host", "127.0.0.1", "--port", str(self.server.port)]
        args.extend(support.FAST_CLI_FLAGS)
        args.extend(extra)
        return support.run_cli(args)

    def test_getvar(self):
        result = self.cli("getvar", "version", "product")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("version: 0.5", result.stdout)
        self.assertIn("product: xaga", result.stdout)
        self.assertIn("Finished. total time:", result.stdout)

    def test_getvar_unknown_exit_code(self):
        result = self.cli("getvar", "no-such-variable")
        self.assertEqual(result.returncode, 1)
        self.assertIn("FAIL", result.stderr)

    def test_flash_option(self):
        image = bytes((i * 13 + 5) & 0xFF for i in range(40000))
        handle = tempfile.NamedTemporaryFile(delete=False)
        try:
            handle.write(image)
            handle.close()
            result = self.cli("--flash", "boot=" + handle.name)
        finally:
            os.unlink(handle.name)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("[flash] boot: OKAY", result.stdout)
        self.assertTrue(self.device_mock.wait_for(lambda: len(self.device_mock.flashes) == 1, 2))
        self.assertEqual(self.device_mock.flashes[0], ("boot", image))

    def test_flash_file_path_with_spaces(self):
        image = b"\x00\x01" * 5000
        directory = tempfile.mkdtemp()
        path = os.path.join(directory, "my boot.img")
        try:
            with open(path, "wb") as handle:
                handle.write(image)
            result = self.cli("flash", "boot_a", path)
        finally:
            os.unlink(path)
            os.rmdir(directory)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(self.device_mock.wait_for(lambda: len(self.device_mock.flashes) == 1, 2))
        self.assertEqual(self.device_mock.flashes[0], ("boot_a", image))

    def test_no_action_returns_usage_error(self):
        result = self.cli()
        self.assertEqual(result.returncode, 2)

    def test_bad_flash_spec_returns_usage_error(self):
        result = self.cli("--flash", "boot_without_equals")
        self.assertEqual(result.returncode, 2)

    def _write_image(self, size):
        image = bytes((i * 11 + 1) & 0xFF for i in range(size))
        handle = tempfile.NamedTemporaryFile(delete=False)
        try:
            handle.write(image)
            handle.close()
        finally:
            pass
        self.addCleanup(lambda: os.path.exists(handle.name) and os.unlink(handle.name))
        return handle.name, image

    def test_ack_pacing_flash(self):
        self.device_mock.ack_frames = True
        path, image = self._write_image(40000)
        result = self.cli("--pacing", "ack", "--ack-window", "2", "flash", "boot", path)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("acks=", result.stdout)
        self.assertTrue(self.device_mock.wait_for(
            lambda: len(self.device_mock.flashes) == 1, 2))
        self.assertEqual(self.device_mock.flashes[0], ("boot", image))

    def test_ack_timeout_prints_diagnostics(self):
        self.device_mock.ack_frames = True
        self.device_mock.ack_drop_after = 1  # command ACK only
        path, _image = self._write_image(40000)
        result = self.cli("--pacing", "ack", "--ack-window", "1",
                          "--ack-timeout", "0.3", "flash", "boot", path)
        self.assertEqual(result.returncode, 1)
        self.assertIn("[diag] failure snapshot", result.stderr)
        self.assertIn("sent ", result.stderr)
        self.assertIn("acks=", result.stderr)

    def test_dump_log_written(self):
        self.device_mock.ack_frames = True
        path, _image = self._write_image(40000)
        dump = os.path.join(tempfile.mkdtemp(), "traffic.log")
        self.addCleanup(lambda: os.path.exists(dump) and os.unlink(dump))
        result = self.cli("--pacing", "ack", "--ack-window", "2",
                          "--dump-log", dump, "flash", "boot", path)
        self.assertEqual(result.returncode, 0, result.stderr)
        with open(dump, "r", encoding="utf-8") as handle:
            content = handle.read()
        self.assertIn("# vusb traffic dump", content)
        self.assertIn("EP1_ACK", content)
        self.assertIn("SOFT_CONN", content)
        self.assertIn("TX    EP1_OUT", content)

    def test_max_download_size_rejects_before_download(self):
        self.device_mock.vars["max-download-size"] = "0x00100000"  # 1 MiB
        path, _image = self._write_image(2 * 1024 * 1024)
        result = self.cli("--flash", "boot=" + path)
        self.assertEqual(result.returncode, 1)
        self.assertIn("max-download-size", result.stderr)
        self.assertEqual(self.device_mock.downloads, [])

    def test_missing_image_fails_before_connecting(self):
        result = self.cli("--flash", "boot=/no/such/boot.img")
        self.assertEqual(result.returncode, 2)
        self.assertIn("image file not found", result.stderr)
        self.assertEqual(self.device_mock.setups, [])


if __name__ == "__main__":
    unittest.main()
