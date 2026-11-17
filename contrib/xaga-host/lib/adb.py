"""adb (USB function-fs) protocol skeleton for the MT6895 virtual UDC.

STATUS: skeleton.  The packet codec and the CNXN handshake are implemented and
unit-tested against MockAdbDevice; the stream layer (AUTH signature, OPEN /
WRTE / OKAY / CLSE multiplexing) is deliberately minimal.  It cannot talk to
a real adbd yet because the guest side is not ready:

  * LK fastboot currently owns the UDC (device mode) and the kernel mtu3
    gadget is not modelled/registered, so /sys/class/udc does not exist yet;
  * once the kernel gadget path works, adbd will expose a *different
    interface* on the same device (class ff/42/01, EP in/out) and the host
    must re-enumerate and select it (TODO: discover endpoints from the
    config descriptor instead of hard-coding EP1).

Protocol notes (AOSP adb_protocol.txt):
  * 24-byte header: command, arg0, arg1, data_length, data_crc32, magic
    (magic = command ^ 0xffffffff, data_crc32 = sum of data bytes & 0xffffffff)
  * CNXN(version, maxdata, "host::features=...") -> CNXN(version, maxdata, banner)
  * If the device does not trust the host key it answers AUTH(TOKEN, 20 bytes)
    and the host must reply AUTH(SIGNATURE, RSA-SHA1(token)) or
    AUTH(RSAPUBLICKEY, pubkey).
  * OPEN(0, 0, "service\\0") -> OKAY(local_id, remote_id), then WRTE/OKAY
    chunks of at most maxdata bytes, CLSE to close.

TODO(adb integration):
  1. AUTH SIGNATURE: Python's standard library has no RSA signing.  Options:
     - shell out to "openssl dgst -sha1 -sign adbkey" (host machine has it), or
     - parse the adb private key (PKCS#1 DER) and do modular exponentiation
       with ints in pure Python (feasible: 2048-bit, ~30 lines).
     Keep the choice local to auth() so the rest of the client does not care.
  2. Stream layer: maintain local_id/remote_id tables, honour maxdata, handle
     interleaved streams, flow control for large writes, and EOF (CLSE).
  3. Transport: the real gadget may use EP2/EP3 (or the same EP1 pair on a
     different interface); add endpoint discovery from UsbDeviceInfo and a
     bulk_in/out adapter for arbitrary endpoints once the model supports it
     (the current mtk_usbdev.c only wires EP1).
  4. Sync service (SEND/RECV/STAT/LIST/DONE) and shell protocol are out of
     scope for the skeleton.
"""
from __future__ import annotations

import dataclasses
import logging
import struct
import time
from typing import Dict, List, Optional, Tuple

from . import vusb

LOG = logging.getLogger("vusb.adb")

# -- protocol constants ------------------------------------------------------
A_VERSION = 0x01000000
A_MAXDATA = 4096

A_SYNC = 0x434E5953
A_CNXN = 0x4E584E43
A_OPEN = 0x4E45504F
A_OKAY = 0x59414B4F
A_CLSE = 0x45534C43
A_WRTE = 0x45545257
A_AUTH = 0x48545541
A_STLS = 0x534C5453

CMD_NAMES = {
    A_SYNC: "SYNC",
    A_CNXN: "CNXN",
    A_OPEN: "OPEN",
    A_OKAY: "OKAY",
    A_CLSE: "CLSE",
    A_WRTE: "WRTE",
    A_AUTH: "AUTH",
    A_STLS: "STLS",
}

AUTH_TOKEN = 1
AUTH_SIGNATURE = 2
AUTH_RSAPUBLICKEY = 3

#: AOSP adb source spells the command constants CMD_*; keep both spellings.
CMD_SYNC = A_SYNC
CMD_CNXN = A_CNXN
CMD_OPEN = A_OPEN
CMD_OKAY = A_OKAY
CMD_CLSE = A_CLSE
CMD_WRTE = A_WRTE
CMD_AUTH = A_AUTH
CMD_STLS = A_STLS

HEADER = struct.Struct("<6I")
MAX_PACKET_DATA = 1024 * 1024
HOST_BANNER = "host::features=cmd,shell_v2,stat_v2"


class AdbError(vusb.VusbError):
    """Base adb client error."""


class AdbProtocolError(AdbError):
    """Malformed or unexpected adb packet."""


def packet_checksum(data: bytes) -> int:
    """adb uses the plain byte sum (not CRC-32) as data checksum."""
    return sum(data) & 0xFFFFFFFF


@dataclasses.dataclass
class AdbPacket:
    command: int
    arg0: int = 0
    arg1: int = 0
    data: bytes = b""

    @property
    def name(self) -> str:
        return CMD_NAMES.get(self.command, "0x%08x" % self.command)

    def encode(self) -> bytes:
        data = bytes(self.data)
        header = HEADER.pack(
            self.command & 0xFFFFFFFF,
            self.arg0 & 0xFFFFFFFF,
            self.arg1 & 0xFFFFFFFF,
            len(data),
            packet_checksum(data),
            (self.command ^ 0xFFFFFFFF) & 0xFFFFFFFF,
        )
        return header + data

    def __str__(self) -> str:  # pragma: no cover - cosmetic
        return "AdbPacket(%s arg0=%d arg1=%d len=%d)" % (
            self.name, self.arg0, self.arg1, len(self.data)
        )


def decode_packet(buffer: bytes) -> Tuple[Optional[AdbPacket], int]:
    """Decode one packet from *buffer*.

    Returns (packet, consumed) or (None, 0) when more bytes are needed.
    Raises AdbProtocolError for corrupt headers.
    """
    buffer = bytes(buffer)
    if len(buffer) < HEADER.size:
        return None, 0
    command, arg0, arg1, length, checksum, magic = HEADER.unpack_from(buffer, 0)
    if magic != (command ^ 0xFFFFFFFF):
        raise AdbProtocolError(
            "bad adb magic 0x%08x for command 0x%08x" % (magic, command)
        )
    if length > MAX_PACKET_DATA:
        raise AdbProtocolError("adb packet length %d too large" % length)
    end = HEADER.size + length
    if len(buffer) < end:
        return None, 0
    data = buffer[HEADER.size:end]
    if checksum != packet_checksum(data):
        raise AdbProtocolError(
            "bad adb checksum 0x%08x != 0x%08x (%d data bytes)"
            % (checksum, packet_checksum(data), len(data))
        )
    return AdbPacket(command, arg0, arg1, data), end


class AdbPacketReader:
    """Incremental packet reader fed with raw bulk-IN bytes."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> None:
        self._buffer.extend(data)

    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)

    def pop_packet(self) -> Optional[AdbPacket]:
        packet, consumed = decode_packet(bytes(self._buffer))
        if packet is None:
            return None
        del self._buffer[:consumed]
        return packet


class AdbClient:
    """Minimal adb client over the virtual UDC bulk endpoints.

    The transport is the same framed chardev used by fastboot; the guest-side
    adbd gadget is not available in the current model, so this class is mainly
    a well-tested skeleton for the future kernel-gadget integration.
    """

    def __init__(
        self,
        usb: Optional[vusb.VUsbHost] = None,
        host: str = "127.0.0.1",
        port: int = 4444,
        *,
        maxdata: int = A_MAXDATA,
        banner: str = HOST_BANNER,
        connect_timeout: float = 30.0,
        io_timeout: float = 10.0,
        soft_conn_timeout: float = 150.0,
        pullup_settle: float = 0.5,
        bus_settle: float = 0.8,
        qmu_timeout: float = 5.0,
        response_idle: float = 0.05,
    ) -> None:
        self.usb = usb if usb is not None else vusb.VUsbHost(
            host, port, connect_timeout=connect_timeout, io_timeout=io_timeout
        )
        self.maxdata = int(maxdata)
        self.banner = banner
        self.soft_conn_timeout = soft_conn_timeout
        self.pullup_settle = pullup_settle
        self.bus_settle = bus_settle
        self.qmu_timeout = qmu_timeout
        self.response_idle = response_idle
        self.device: Optional[vusb.UsbDeviceInfo] = None
        self._reader = AdbPacketReader()
        self._next_local_id = 1
        self.pending: List[AdbPacket] = []

    # -- lifecycle ----------------------------------------------------------
    def connect(self) -> vusb.UsbDeviceInfo:
        """Bring up the UDC (same handshake as fastboot) and enumerate."""
        self.usb.connect()
        self.usb.wait_ready(5.0)
        try:
            self.usb.wait_pullup(timeout=self.soft_conn_timeout, settle=self.pullup_settle)
        except vusb.VusbTimeout as exc:
            LOG.warning("%s; probing enumeration anyway", exc)
        self.usb.connect_bus(wait_connected=1.0)
        if self.bus_settle > 0:
            self.usb.pump(self.bus_settle)
        self.device = self.usb.enumerate(configure=True)
        if not self.usb.wait_log("QMU_READY", self.qmu_timeout):
            LOG.warning("no LOG QMU_READY (fine for a kernel gadget)")
        LOG.info("adb gadget enumerated: %s", self.device.summary())
        return self.device

    def close(self) -> None:
        self.usb.close()

    def __enter__(self) -> "AdbClient":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # -- packet transport ---------------------------------------------------
    def send_packet(self, packet: AdbPacket) -> None:
        LOG.debug("-> %s", packet)
        self.usb.bulk_out_raw(packet.encode())

    def recv_packet(self, timeout: float = 5.0) -> AdbPacket:
        deadline = time.monotonic() + float(timeout)
        while True:
            if self.pending:
                return self.pending.pop(0)
            packet = self._reader.pop_packet()
            if packet is not None:
                LOG.debug("<- %s", packet)
                return packet
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise vusb.VusbTimeout("timed out waiting for an adb packet")
            raw = self.usb.bulk_in(timeout=remaining, idle_gap=self.response_idle)
            self._reader.feed(raw)

    # -- handshake ----------------------------------------------------------
    def cnxn(self, *, timeout: float = 5.0) -> AdbPacket:
        """Send CNXN and return the device response (CNXN or AUTH).

        A CNXN reply means the device accepted the host without signing;
        an AUTH(TOKEN) reply must be followed by :meth:'auth' (TODO).
        """
        self.send_packet(
            AdbPacket(A_CNXN, A_VERSION, self.maxdata, self.banner.encode("ascii"))
        )
        response = self.recv_packet(timeout=timeout)
        if response.command == A_AUTH:
            LOG.warning(
                "device requested AUTH (token %s); RSA signing is not implemented yet",
                response.data.hex(),
            )
        elif response.command != A_CNXN:
            raise AdbProtocolError("expected CNXN/AUTH, got %s" % response)
        return response

    def auth(self, token: bytes = b"", *, timeout: float = 5.0) -> AdbPacket:
        """AUTH signature step (TODO: RSA-SHA1 signing not implemented)."""
        raise NotImplementedError(
            "adb AUTH signature is not implemented yet; mock device runs with "
            "auth disabled.  TODO: sign token with the adb private key "
            "(openssl subprocess or pure-python RSA) and send AUTH(SIGNATURE)."
        )

    # -- streams (minimal) --------------------------------------------------
    def open(self, service: str, *, timeout: float = 5.0) -> Tuple[int, int]:
        """Open a service; returns (local_id, remote_id) after OKAY."""
        local_id = self._next_local_id
        self._next_local_id += 1
        self.send_packet(AdbPacket(A_OPEN, local_id, 0, service.encode("utf-8") + b"\x00"))
        deadline = time.monotonic() + float(timeout)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise vusb.VusbTimeout("timed out waiting for OPEN OKAY")
            response = self.recv_packet(timeout=remaining)
            if response.command == A_OKAY:
                return local_id, response.arg1
            if response.command == A_CLSE:
                raise AdbError("device refused to open service %r" % service)
            self.pending.append(response)

    def write(self, local_id: int, remote_id: int, data: bytes, *, timeout: float = 5.0) -> int:
        """Send data over an open stream; returns the next local id hint."""
        chunk_size = max(1, self.maxdata)
        offset = 0
        while offset < len(data):
            chunk = data[offset:offset + chunk_size]
            self.send_packet(AdbPacket(A_WRTE, local_id, remote_id, chunk))
            self.recv_packet(timeout=timeout)  # OKAY
            offset += len(chunk)
        return local_id

    def close_stream(self, local_id: int, remote_id: int) -> None:
        self.send_packet(AdbPacket(A_CLSE, local_id, remote_id))
