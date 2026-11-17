"""Frame codec + USB descriptor parser unit tests (no sockets)."""
from __future__ import annotations

import os
import sys
import unittest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
HOST_DIR = os.path.dirname(TESTS_DIR)
for _path in (HOST_DIR, TESTS_DIR):
    if _path not in sys.path:
        sys.path.insert(0, _path)

import support  # noqa: E402
from lib import vusb  # noqa: E402



class FrameCodecTests(unittest.TestCase):
    def test_roundtrip_single_frame(self):
        raw = vusb.encode_frame(vusb.HOST_EP1_OUT, b"getvar:version")
        self.assertEqual(raw[0], vusb.HOST_EP1_OUT)
        self.assertEqual(raw[1] | (raw[2] << 8), 14)
        decoder = vusb.FrameDecoder()
        frames = decoder.feed(raw)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].type, vusb.HOST_EP1_OUT)
        self.assertEqual(frames[0].payload, b"getvar:version")
        self.assertEqual(decoder.buffered_bytes, 0)

    def test_empty_payload(self):
        decoder = vusb.FrameDecoder()
        frames = decoder.feed(vusb.encode_frame(vusb.HOST_CONNECT))
        self.assertEqual([(f.type, f.payload) for f in frames], [(vusb.HOST_CONNECT, b"")])

    def test_fragmented_stream(self):
        raw = vusb.encode_frame(vusb.DEV_LOG, b"SOFT_CONN") + vusb.encode_frame(
            vusb.DEV_EP0_IN, b"\x12\x01\x00\x02"
        )
        decoder = vusb.FrameDecoder()
        collected = []
        for byte in raw:  # one byte at a time
            collected.extend(decoder.feed(bytes([byte])))
        self.assertEqual([f.type for f in collected], [vusb.DEV_LOG, vusb.DEV_EP0_IN])
        self.assertEqual(collected[0].payload, b"SOFT_CONN")
        self.assertEqual(collected[1].payload, b"\x12\x01\x00\x02")
        self.assertEqual(decoder.buffered_bytes, 0)

    def test_partial_frame_waits_for_more(self):
        raw = vusb.encode_frame(vusb.DEV_EP1_IN, b"OKAY0.5")
        decoder = vusb.FrameDecoder()
        self.assertEqual(decoder.feed(raw[:4]), [])
        self.assertGreater(decoder.buffered_bytes, 0)
        frames = decoder.feed(raw[4:])
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].payload, b"OKAY0.5")

    def test_oversize_payload_rejected(self):
        with self.assertRaises(ValueError):
            vusb.encode_frame(vusb.HOST_EP1_OUT, b"\x00" * (vusb.MAX_FRAME_PAYLOAD + 1))
        with self.assertRaises(ValueError):
            vusb.Frame(vusb.HOST_EP1_OUT, b"\x00" * (vusb.MAX_FRAME_PAYLOAD + 1))

    def test_frame_names(self):
        self.assertEqual(vusb.Frame(vusb.HOST_SETUP).name, "SETUP")
        self.assertEqual(vusb.Frame(vusb.DEV_LOG).name, "LOG")
        self.assertEqual(vusb.Frame(0x33).name, "0x33")
        self.assertIn("len=1", repr(vusb.Frame(vusb.DEV_EP1_IN, b"x")))


class DescriptorTests(unittest.TestCase):
    #: Descriptor bytes captured from a real LK fastboot run (r1295).
    DEVICE = bytes.fromhex("1201000200000040d1180dd0000101020301")
    CONFIG = bytes.fromhex(
        "0902200001010080800904000002ff4203040705810200020107050102000200"
    )

    def test_device_descriptor(self):
        desc = vusb.DeviceDescriptor.parse(self.DEVICE)
        self.assertEqual(desc.idVendor, 0x18D1)
        self.assertEqual(desc.idProduct, 0xD00D)
        self.assertEqual(desc.bcdUSB, 0x0200)
        self.assertEqual(desc.bMaxPacketSize0, 64)
        self.assertEqual(desc.bNumConfigurations, 1)
        self.assertEqual(desc.raw(), self.DEVICE)

    def test_config_descriptor_endpoints(self):
        config = vusb.ConfigDescriptor.parse(self.CONFIG)
        self.assertEqual(config.wTotalLength, 32)
        self.assertEqual(len(config.raw), 32)
        self.assertEqual(len(config.interfaces), 1)
        self.assertEqual(config.bConfigurationValue, 1)
        endpoints = config.endpoints
        self.assertEqual([ep.bEndpointAddress for ep in endpoints], [0x81, 0x01])
        self.assertTrue(endpoints[0].direction_in)
        self.assertFalse(endpoints[1].direction_in)
        self.assertEqual(endpoints[0].transfer_type, vusb.EP_TYPE_BULK)
        bulk = config.find_bulk_endpoints()
        self.assertEqual(bulk["in"].wMaxPacketSize, 512)
        self.assertEqual(bulk["out"].bEndpointAddress, 0x01)

    def test_truncated_descriptors_raise(self):
        with self.assertRaises(vusb.ProtocolError):
            vusb.DeviceDescriptor.parse(b"\x12\x01\x00")
        with self.assertRaises(vusb.ProtocolError):
            vusb.ConfigDescriptor.parse(b"\x09\x02")

    def test_string_descriptor(self):
        raw = b"\x0a\x03" + "xaga".encode("utf-16-le")
        self.assertEqual(vusb.parse_string_descriptor(raw, 2), "xaga")
        lang = vusb.parse_string_descriptor(bytes([4, 3, 0x09, 0x04]), 0)
        self.assertEqual(lang, "0x0409")

    def test_transfer_stats(self):
        stats = vusb.TransferStats(byte_count=1024 * 1024, chunk_count=64, seconds=0.5)
        self.assertAlmostEqual(stats.bytes_per_second, 2.0 * 1024 * 1024)
        self.assertIn("MiB", str(stats))


if __name__ == "__main__":
    unittest.main()
