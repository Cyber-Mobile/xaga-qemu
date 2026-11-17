"""adb protocol tests: packet codec, RSA/AUTH, streams, shell, sideload, push."""
import base64
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
HOST_DIR = os.path.dirname(TESTS_DIR)
for _path in (HOST_DIR, TESTS_DIR):
    if _path not in sys.path:
        sys.path.insert(0, _path)

import support  # noqa: E402
from lib import adb  # noqa: E402


class AdbPacketTests(unittest.TestCase):
    def test_roundtrip(self):
        packet = adb.AdbPacket(adb.A_CNXN, adb.A_VERSION, 4096, b"host::features=x")
        raw = packet.encode()
        self.assertEqual(len(raw), 24 + len(packet.data))
        decoded, consumed = adb.decode_packet(raw)
        self.assertEqual(consumed, len(raw))
        self.assertEqual(decoded.command, adb.A_CNXN)
        self.assertEqual(decoded.arg0, adb.A_VERSION)
        self.assertEqual(decoded.arg1, 4096)
        self.assertEqual(decoded.data, b"host::features=x")
        self.assertEqual(decoded.encode(), raw)

    def test_incomplete_packet_returns_none(self):
        raw = adb.AdbPacket(adb.A_WRTE, 1, 2, b"hello").encode()
        self.assertEqual(adb.decode_packet(raw[:10]), (None, 0))
        self.assertEqual(adb.decode_packet(raw[:24]), (None, 0))

    def test_bad_magic_raises(self):
        raw = bytearray(adb.AdbPacket(adb.A_OKAY, 1, 2).encode())
        raw[20] ^= 0xFF
        with self.assertRaises(adb.AdbProtocolError):
            adb.decode_packet(bytes(raw))

    def test_bad_checksum_raises(self):
        raw = bytearray(adb.AdbPacket(adb.A_WRTE, 1, 2, b"hello").encode())
        raw[24] ^= 0xFF
        with self.assertRaises(adb.AdbProtocolError):
            adb.decode_packet(bytes(raw))

    def test_packet_reader_fragmented(self):
        first = adb.AdbPacket(adb.A_OKAY, 1, 101).encode()
        second = adb.AdbPacket(adb.A_WRTE, 1, 101, b"data").encode()
        reader = adb.AdbPacketReader()
        reader.feed(first[:7])
        self.assertIsNone(reader.pop_packet())
        reader.feed(first[7:] + second)
        self.assertEqual(reader.pop_packet().command, adb.A_OKAY)
        self.assertEqual(reader.pop_packet().data, b"data")
        self.assertIsNone(reader.pop_packet())

    def test_banner_parse(self):
        banner = adb.AdbBanner.parse(
            b"recovery::ro.product.device=xaga;ro.serialno=ABC;features=cmd,shell_v2"
        )
        self.assertEqual(banner.state, "recovery")
        self.assertEqual(banner.device_name, "xaga")
        self.assertEqual(banner.serial, "ABC")
        self.assertEqual(banner.features, {"cmd", "shell_v2"})


class AdbCryptoTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.key = adb.RsaKey.load(support.TEST_KEY_PATH)

    def test_pkcs1_and_pkcs8_keys_match(self):
        key8 = adb.RsaKey.load(support.TEST_KEY_PKCS8_PATH)
        self.assertEqual(key8.n, self.key.n)
        self.assertEqual(key8.e, self.key.e)
        self.assertEqual(self.key.bits, 2048)

    def test_sign_verify_roundtrip(self):
        token = bytes(range(20))
        signature = self.key.sign_sha1(token)
        self.assertEqual(len(signature), 256)
        self.assertTrue(self.key.verify_sha1(token, signature))
        self.assertFalse(self.key.verify_sha1(token + b"x", signature))
        tampered = bytearray(signature)
        tampered[-1] ^= 0x01
        self.assertFalse(self.key.verify_sha1(token, bytes(tampered)))

    @unittest.skipUnless(shutil.which("openssl"), "openssl not available")
    def test_signature_matches_openssl(self):
        token = b"a test adb auth token"
        ours = self.key.sign_sha1(token)
        openssl = shutil.which("openssl")
        result = subprocess.run(
            [openssl, "dgst", "-sha1", "-sign", support.TEST_KEY_PATH],
            input=token, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
        )
        self.assertEqual(result.stdout, ours)

    def test_android_public_key_roundtrip(self):
        text = self.key.adb_public_key()
        blob = base64.b64decode(text)
        self.assertEqual(len(blob), adb.ANDROID_PUBKEY_ENCODED_SIZE)
        self.assertEqual(len(blob), 524)
        n, e = adb.decode_adb_public_key(text)
        self.assertEqual(n, self.key.n)
        self.assertEqual(e, self.key.e)
        # Pre-computed Montgomery parameters must match the AOSP layout:
        # modulus_size_words, n0inv, modulus LE, rr LE, exponent LE.
        words, n0inv = __import__("struct").unpack_from("<II", blob, 0)
        self.assertEqual(words, 64)
        r32 = 1 << 32
        self.assertEqual(n0inv, (-pow(self.key.n & (r32 - 1), -1, r32)) % r32)
        modulus = blob[8:8 + 256]
        rr_bytes = blob[8 + 256:8 + 512]
        self.assertEqual(int.from_bytes(modulus, "little"), self.key.n)
        self.assertEqual(int.from_bytes(rr_bytes, "little"),
                         pow(2, 2 * 8 * 256, self.key.n))

    def test_public_numbers_match_openssl_public_pem(self):
        with open(support.TEST_PUB_PATH, "r", encoding="utf-8") as handle:
            pem = handle.read()
        der = base64.b64decode("".join(
            line for line in pem.splitlines() if "-----" not in line
        ))
        n, e = adb.rsa_public_numbers_from_der(der)
        self.assertEqual(n, self.key.n)
        self.assertEqual(e, self.key.e)

    @unittest.skipUnless(shutil.which("openssl"), "openssl not available")
    def test_generate_key(self):
        directory = tempfile.mkdtemp()
        try:
            path = os.path.join(directory, "adbkey")
            key = adb.RsaKey.generate(path)
            self.assertTrue(os.path.exists(path))
            loaded = adb.RsaKey.load(path)
            token = b"generated key token"
            self.assertTrue(loaded.verify_sha1(token, loaded.sign_sha1(token)))
        finally:
            shutil.rmtree(directory, ignore_errors=True)


class AdbHandshakeTests(support.MockAdbCase):
    def test_no_auth_handshake(self):
        self.connect()
        self.assertIsNotNone(self.client.banner)
        self.assertEqual(self.client.banner.state, "device")
        self.assertEqual(self.client.banner.serial, "xaga")
        self.assertEqual(self.client.auth_used, "none")
        self.assertEqual(self.client.maxdata, 4096)
        self.assertIn("shell_v2", self.client.features)
        self.assertIn("cnxn_accepted", self.device_mock.auth_events)

    def test_cnxn_packet_style_compat(self):
        self.client.connect()
        response = self.client.cnxn(timeout=5.0)
        self.assertEqual(response.command, adb.A_CNXN)
        self.assertEqual(response.arg0, adb.A_VERSION)
        banner = adb.AdbBanner.parse(response.data, response.arg0, response.arg1)
        self.assertIn("features=", response.data.decode("latin-1"))
        self.assertEqual(banner.device_name, "xaga")

    def test_auth_signature_accepted(self):
        key = self.load_test_key()
        self.client.key_path = support.TEST_KEY_PATH
        self.device_mock.auth_mode = "token"
        self.device_mock.authorized_keys = [key.public_numbers()]
        self.connect()
        self.assertEqual(self.client.auth_used, "signature")
        self.assertIn("signature_ok", self.device_mock.auth_events)

    def test_auth_public_key_upload(self):
        self.client.key_path = support.TEST_KEY_PATH
        self.device_mock.auth_mode = "token"
        self.device_mock.authorized_keys = []
        self.connect()
        self.assertEqual(self.client.auth_used, "publickey")
        self.assertIn("signature_rejected", self.device_mock.auth_events)
        self.assertIn("publickey", self.device_mock.auth_events)
        self.assertEqual(len(self.device_mock.authorized_keys), 1)

    def test_auth_rejected_raises(self):
        self.client.key_path = support.TEST_KEY_PATH
        self.device_mock.auth_mode = "token"
        self.device_mock.authorized_keys = []
        self.device_mock.accept_public_key_upload = False
        with self.assertRaises(adb.AdbAuthError):
            self.client.connect()
            self.client.handshake(timeout=5.0)
        self.assertIn("signature_rejected", self.device_mock.auth_events)
        self.assertIn("publickey", self.device_mock.auth_events)

    @unittest.skipUnless(shutil.which("openssl"), "openssl not available")
    def test_auth_key_auto_generated(self):
        directory = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, directory, True)
        self.client.key_path = os.path.join(directory, "adbkey")
        self.device_mock.auth_mode = "token"
        self.device_mock.authorized_keys = []
        self.connect()
        self.assertTrue(os.path.exists(self.client.key_path))
        self.assertEqual(self.client.auth_used, "publickey")


class AdbShellTests(support.MockAdbCase):
    def test_shell_v2_stdout_and_exit(self):
        self.connect()
        result = self.client.shell("getprop ro.product.device", timeout=10)
        self.assertEqual(result.protocol, "v2")
        self.assertEqual(result.stdout, b"xaga\n")
        self.assertEqual(result.exit_code, 0)

    def test_shell_legacy(self):
        self.connect()
        result = self.client.shell("echo hello", timeout=10, protocol="legacy")
        self.assertEqual(result.protocol, "legacy")
        self.assertEqual(result.stdout, b"hello\n")
        self.assertIsNone(result.exit_code)

    def test_shell_v2_stderr_and_exit_code(self):
        self.device_mock.shell_handler = lambda command: adb.ShellResult(b"out", b"err", 7)
        self.connect()
        result = self.client.shell("whatever", timeout=10)
        self.assertEqual(result.stdout, b"out")
        self.assertEqual(result.stderr, b"err")
        self.assertEqual(result.exit_code, 7)

    def test_shell_auto_falls_back_without_shell_v2_feature(self):
        self.device_mock.features = ("cmd",)
        self.connect()
        self.assertNotIn("shell_v2", self.client.features)
        result = self.client.shell("getprop ro.product.device", timeout=10)
        self.assertEqual(result.protocol, "legacy")
        self.assertEqual(result.stdout, b"xaga\n")


class AdbRebootTests(support.MockAdbCase):
    def test_reboot_default_and_targets(self):
        self.connect()
        self.client.reboot("", timeout=10)
        self.assertEqual(self.device_mock.reboot_target, "")
        self.client.reboot("recovery", timeout=10)
        self.assertEqual(self.device_mock.reboot_target, "recovery")
        self.client.reboot("bootloader", timeout=10)
        self.assertEqual(self.device_mock.reboot_target, "bootloader")

    def test_reboot_invalid_target(self):
        with self.assertRaises(ValueError):
            self.client.reboot("banana")


class AdbSideloadTests(support.MockAdbCase):
    def _package(self, size, seed=1):
        path, data = support.make_temp_package(size, seed)
        self.addCleanup(lambda: os.path.exists(path) and os.unlink(path))
        return path, data

    def test_sideload_host_e2e(self):
        path, data = self._package(300000)
        self.device_mock.expect_sideload(data)
        self.connect()
        calls = []
        stats = self.client.sideload(
            path, progress=calls.append, timeout=30, block_size=65536
        )
        self.assertEqual(stats.protocol, "sideload-host")
        self.assertTrue(stats.ok)
        self.assertEqual(stats.bytes_served, len(data))
        self.assertEqual(stats.blocks, 5)
        self.assertEqual(len(calls), 5)
        self.assertGreater(stats.percent, 0.0)
        self.assertTrue(self.device_mock.wait_for(
            lambda: self.device_mock.sideload_ok is not None, 3))
        self.assertTrue(self.device_mock.sideload_ok, self.device_mock.sideload_error)
        import hashlib
        self.assertEqual(self.device_mock.sideload_sha256, hashlib.sha256(data).hexdigest())
        self.assertEqual(self.device_mock.sideload_bytes, len(data))

    def test_sideload_block_boundary(self):
        path, data = self._package(2 * 65536)
        self.device_mock.expect_sideload(data)
        self.connect()
        stats = self.client.sideload(path, timeout=30)
        self.assertEqual(stats.blocks, 2)
        self.assertEqual(stats.bytes_served, 2 * 65536)

    def test_sideload_device_fail_raises(self):
        path, data = self._package(200000)
        self.device_mock.expect_sideload(data)
        self.device_mock.sideload_fail = True
        self.connect()
        with self.assertRaises(adb.SideloadError) as ctx:
            self.client.sideload(path, timeout=30)
        self.assertIn("FAILFAIL", str(ctx.exception))
        self.assertTrue(self.device_mock.wait_for(
            lambda: self.device_mock.sideload_ok is not None, 3))
        self.assertFalse(self.device_mock.sideload_ok)

    def test_sideload_hash_mismatch_raises(self):
        path, data = self._package(200000)
        self.device_mock.expected_sideload_size = len(data)
        self.device_mock.expected_sideload_sha256 = "0" * 64
        self.connect()
        with self.assertRaises(adb.SideloadError):
            self.client.sideload(path, timeout=30)
        self.assertTrue(self.device_mock.wait_for(
            lambda: self.device_mock.sideload_ok is not None, 3))
        self.assertFalse(self.device_mock.sideload_ok)

    def test_legacy_sideload_fallback(self):
        path, data = self._package(150000)
        self.device_mock.legacy_only = True
        self.device_mock.expect_sideload(data)
        self.connect()
        stats = self.client.sideload(path, timeout=30)
        self.assertEqual(stats.protocol, "sideload")
        self.assertEqual(stats.bytes_served, len(data))
        self.assertTrue(self.device_mock.wait_for(
            lambda: self.device_mock.sideload_ok is not None, 3))
        self.assertTrue(self.device_mock.sideload_ok, self.device_mock.sideload_error)

    def test_legacy_fallback_disabled_raises(self):
        path, _data = self._package(70000)
        self.device_mock.legacy_only = True
        self.connect()
        with self.assertRaises(adb.SideloadError):
            self.client.sideload(path, timeout=30, legacy_fallback=False)

    def test_sideload_missing_file_raises(self):
        with self.assertRaises(adb.SideloadError):
            self.client.sideload("/no/such/package.zip", timeout=5)

    @unittest.skipUnless(
        os.environ.get("VUSB_BIG_TEST"),
        "set VUSB_BIG_TEST=1 to run the 64 MiB sideload regression",
    )
    def test_sideload_64mb_optional(self):
        path, data = self._package(64 * 1024 * 1024)
        self.device_mock.expect_sideload(data)
        self.connect()
        stats = self.client.sideload(path, timeout=120)
        self.assertEqual(stats.bytes_served, len(data))
        self.assertTrue(self.device_mock.wait_for(
            lambda: self.device_mock.sideload_ok is not None, 10))
        self.assertTrue(self.device_mock.sideload_ok)


class AdbPushTests(support.MockAdbCase):
    def test_push_file(self):
        path, data = support.make_temp_package(150000)
        self.addCleanup(lambda: os.path.exists(path) and os.unlink(path))
        self.connect()
        pushed = self.client.push(path, "/data/local/tmp/package.bin", timeout=30)
        self.assertEqual(pushed, len(data))
        self.assertTrue(self.device_mock.wait_for(
            lambda: "/data/local/tmp/package.bin" in self.device_mock.sync_received, 3))
        self.assertEqual(
            self.device_mock.sync_received["/data/local/tmp/package.bin"], data
        )


if __name__ == "__main__":
    unittest.main()
