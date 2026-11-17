"""adb_cli.py subprocess tests against the mock adb gadget."""
import os
import shutil
import sys
import tempfile
import unittest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
HOST_DIR = os.path.dirname(TESTS_DIR)
for _path in (HOST_DIR, TESTS_DIR):
    if _path not in sys.path:
        sys.path.insert(0, _path)

import support  # noqa: E402


class AdbCliTests(support.MockAdbCase):
    def cli(self, *extra):
        args = ["--host", "127.0.0.1", "--port", str(self.server.port)]
        args.extend(support.FAST_ADB_CLI_FLAGS)
        args.extend(extra)
        return support.run_adb_cli(args)

    def test_devices(self):
        result = self.cli("devices")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("xaga\tdevice", result.stdout)

    def test_shell(self):
        result = self.cli("shell", "getprop ro.product.device")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, "xaga\n")

    def test_shell_exit_code(self):
        from lib import adb

        self.device_mock.shell_handler = lambda command: adb.ShellResult(
            b"out\n", b"", 7
        )
        result = self.cli("shell", "whatever")
        self.assertEqual(result.returncode, 7, result.stderr)
        self.assertEqual(result.stdout, "out\n")

    def test_reboot_recovery(self):
        result = self.cli("reboot", "recovery")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(self.device_mock.wait_for(
            lambda: self.device_mock.reboot_target == "recovery", 3))

    def test_sideload(self):
        path, data = support.make_temp_package(200000)
        self.addCleanup(lambda: os.path.exists(path) and os.unlink(path))
        self.device_mock.expect_sideload(data)
        result = self.cli("sideload", path)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Total xfer:", result.stdout)
        self.assertTrue(self.device_mock.wait_for(
            lambda: self.device_mock.sideload_ok is not None, 5))
        self.assertTrue(self.device_mock.sideload_ok, self.device_mock.sideload_error)
        self.assertEqual(self.device_mock.sideload_bytes, len(data))

    def test_sideload_missing_file(self):
        result = self.cli("sideload", "/no/such/package.zip")
        self.assertEqual(result.returncode, 2)
        self.assertIn("package not found", result.stderr)

    def test_push(self):
        path, data = support.make_temp_package(120000)
        self.addCleanup(lambda: os.path.exists(path) and os.unlink(path))
        result = self.cli("push", path, "/data/local/tmp/pkg.bin")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(self.device_mock.wait_for(
            lambda: "/data/local/tmp/pkg.bin" in self.device_mock.sync_received, 5))
        self.assertEqual(
            self.device_mock.sync_received["/data/local/tmp/pkg.bin"], data
        )

    def test_devices_with_auth_signature(self):
        from lib import adb

        key = self.load_test_key()
        self.device_mock.auth_mode = "token"
        self.device_mock.authorized_keys = [key.public_numbers()]
        result = self.cli("--key", support.TEST_KEY_PATH, "devices")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("xaga\tdevice", result.stdout)
        self.assertIn("signature_ok", self.device_mock.auth_events)


if __name__ == "__main__":
    unittest.main()
