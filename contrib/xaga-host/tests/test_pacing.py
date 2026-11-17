"""ACK-driven pacing tests (model 0x83 RX-GPD completion frames)."""
import os
import sys
import unittest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
HOST_DIR = os.path.dirname(TESTS_DIR)
for _path in (HOST_DIR, TESTS_DIR):
    if _path not in sys.path:
        sys.path.insert(0, _path)

import support  # noqa: E402
from lib import fastboot, mock_device  # noqa: E402


class AckPacingTests(support.MockFastbootCase):
    def setUp(self):
        super().setUp()
        self.device_mock.ack_frames = True
        self.client.pacing = "ack"
        self.client.ack_window = 2
        self.client.ack_timeout = 2.0

    def test_download_with_ack_pacing(self):
        self.connect()
        image = bytes((i * 19 + 3) & 0xFF for i in range(40000))
        calls = []
        stats = self.client.download(
            image, progress=lambda done, total, elapsed: calls.append(done)
        )
        self.assertEqual(stats.byte_count, 40000)
        self.assertEqual(stats.chunk_count, 3)
        self.assertGreaterEqual(stats.acks, 3)
        # The command frame itself is ACKed before the DATA prompt, so the
        # total ACK count is one higher than the payload ACKs in stats.
        self.assertEqual(stats.acks + 1, self.client.usb.ack_count)
        self.assertTrue(self.device_mock.wait_for(
            lambda: len(self.device_mock.downloads) == 1, 2.0))
        self.assertEqual(self.device_mock.downloads[0], image)
        self.assertEqual(calls[-1], 40000)

    def test_ack_frames_carry_payload(self):
        self.connect()
        self.client.download(b"x" * 100)
        ack = self.client.usb.last_ack
        self.assertIsNotNone(ack)
        self.assertEqual(len(ack.payload), 4)
        self.assertEqual(ack.counter(), ack.index + 1)

    def test_ack_timeout_raises_diagnostics(self):
        # 1 ACK for the download: command frame + 1 for the first payload chunk
        self.device_mock.ack_drop_after = 2
        self.client.ack_window = 1
        self.client.ack_timeout = 0.4
        self.connect()
        image = bytes(range(256)) * 200  # 3*16KiB chunks
        with self.assertRaises(fastboot.FastbootDownloadError) as ctx:
            self.client.download(image)
        details = ctx.exception.details
        self.assertIsNotNone(details)
        self.assertEqual(details.stage, "timeout")
        self.assertEqual(details.pacing, "ack")
        self.assertEqual(details.window, 1)
        self.assertEqual(details.sent_bytes, 2 * 16384)
        self.assertEqual(details.chunks_sent, 2)
        self.assertEqual(details.acks_received, 1)
        self.assertIn("ACK", str(ctx.exception))

    def test_empty_ack_payload_mode(self):
        self.device_mock.ack_payload_mode = "empty"
        self.connect()
        stats = self.client.download(b"y" * 40000)
        self.assertGreaterEqual(stats.acks, 3)


class AckPacingFailTests(support.MockFastbootCase):
    def test_ack_pacing_still_propagates_device_fail(self):
        self.device_mock.ack_frames = True
        self.device_mock.fail_download = True
        self.client.pacing = "ack"
        self.client.ack_window = 2
        self.connect()
        with self.assertRaises(fastboot.FastbootFail) as ctx:
            self.client.download(b"z" * 40000)
        self.assertIn("Read USB error", ctx.exception.message)
        self.assertIsNotNone(ctx.exception.details)
        self.assertEqual(ctx.exception.details.stage, "device-FAIL")


if __name__ == "__main__":
    unittest.main()
