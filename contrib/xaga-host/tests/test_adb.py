"""adb packet codec + CNXN handshake tests against the mock adb gadget."""
import os
import sys
import unittest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
HOST_DIR = os.path.dirname(TESTS_DIR)
for _path in (HOST_DIR, TESTS_DIR):
    if _path not in sys.path:
        sys.path.insert(0, _path)

import support  # noqa: E402
from lib import adb, mock_device  # noqa: E402


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
        raw[20] ^= 0xFF  # magic field
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


class AdbHandshakeTests(unittest.TestCase):
    def setUp(self):
        self.server = mock_device.MockChardevServer(mock_device.MockAdbDevice).start()
        self.addCleanup(self.server.stop)
        self.client = adb.AdbClient(
            host="127.0.0.1",
            port=self.server.port,
            soft_conn_timeout=5.0,
            pullup_settle=0.02,
            bus_settle=0.02,
            qmu_timeout=2.0,
            io_timeout=2.0,
            response_idle=0.02,
        )
        self.addCleanup(self.client.close)

    def connect(self):
        return self.client.connect()

    def test_cnxn_handshake(self):
        self.connect()
        response = self.client.cnxn(timeout=3.0)
        self.assertEqual(response.command, adb.A_CNXN)
        self.assertEqual(response.arg0, adb.A_VERSION)
        self.assertEqual(response.arg1, adb.A_MAXDATA)
        banner = response.data.decode("latin-1")
        self.assertIn("ro.product.device=xaga", banner)
        self.assertIn("features=", banner)
        self.assertTrue(self.server.device.services == [])

    def test_open_okay(self):
        self.connect()
        self.client.cnxn(timeout=3.0)
        local_id, remote_id = self.client.open("shell:id", timeout=3.0)
        self.assertGreater(local_id, 0)
        self.assertGreater(remote_id, 0)
        self.assertEqual(self.server.device.services, ["shell:id"])

    def test_write_is_acked(self):
        self.connect()
        self.client.cnxn(timeout=3.0)
        local_id, remote_id = self.client.open("shell:id", timeout=3.0)
        self.client.write(local_id, remote_id, b"echo hi\n", timeout=3.0)
        self.assertEqual(self.server.device.writes, [b"echo hi\n"])

    def test_auth_signature_is_todo(self):
        with self.assertRaises(NotImplementedError):
            self.client.auth(b"token")

    def test_token_auth_mode_returns_auth_packet(self):
        self.server.device.auth_mode = "token"
        self.connect()
        response = self.client.cnxn(timeout=3.0)
        self.assertEqual(response.command, adb.A_AUTH)
        self.assertEqual(len(response.data), 20)


if __name__ == "__main__":
    unittest.main()
