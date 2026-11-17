"""Virtual USB host for the MT6895 QEMU "virtual UDC" chardev bridge.

The QEMU machine model (hw/misc/mtk_usbdev.c) exposes the emulated mtu3 UDC
over a QEMU chardev socket with a tiny framed protocol:

    frame := u8 type | u16le payload_len | payload

    host -> device
        0x01 SETUP        8-byte USB setup packet
        0x02 EP1_BULK_OUT raw bytes for the guest bulk-OUT endpoint (EP1)
        0x03 EP0_IN_ACK   acknowledge an EP0 IN data stage (no-op in model)
        0x10 HOST_CONNECT host drives bus reset / connect
        0x11 HOST_DISCONNECT
    device -> host
        0x81 EP0_IN        control-IN data (one frame per guest EP0 TX packet)
        0x82 EP1_IN        bulk-IN data (one frame per completed TX GPD)
        0x90 LOG           ASCII status line (READY / SOFT_CONN / ...)

Typical bring-up (see fastboot.FastbootClient for the full LK sequence):

    usb = VUsbHost(host, port)
    usb.connect()
    usb.wait_pullup()          # wait for LOG "SOFT_CONN" (LK pulled D+ up)
    usb.connect_bus()          # HOST_CONNECT -> guest sees a bus reset
    dev = usb.enumerate()      # GET_DESCRIPTOR / SET_ADDRESS / SET_CONFIGURATION
    ...
    usb.bulk_out(b"getvar:version")    # fastboot command
    usb.bulk_in()                      # fastboot reply ("OKAY0.5")

Only the Python standard library is used, so the module also runs on the
Windows/MSYS2 python used by the build machine.
"""
from __future__ import annotations

import dataclasses
import logging
import select
import socket
import struct
import time
from collections import deque
from typing import Callable, Deque, Dict, List, Optional

LOG = logging.getLogger("vusb")

# ---------------------------------------------------------------------------
# framed chardev protocol
# ---------------------------------------------------------------------------
FRAME_HEADER = struct.Struct("<BH")
MAX_FRAME_PAYLOAD = 0xFFFF

HOST_SETUP = 0x01
HOST_EP1_OUT = 0x02
HOST_EP0_ACK = 0x03
HOST_CONNECT = 0x10
HOST_DISCONNECT = 0x11

DEV_EP0_IN = 0x81
DEV_EP1_IN = 0x82
#: Model -> host RX-completion ACK (one frame per completed guest RX GPD).
#: Frame type/payload are defined by the QEMU UDC model; the host
#: only counts frames for flow control, so any payload works.
DEV_EP1_ACK = 0x83
DEV_LOG = 0x90

FRAME_NAMES = {
    HOST_SETUP: "SETUP",
    HOST_EP1_OUT: "EP1_OUT",
    HOST_EP0_ACK: "EP0_ACK",
    HOST_CONNECT: "HOST_CONNECT",
    HOST_DISCONNECT: "HOST_DISCONNECT",
    DEV_EP0_IN: "EP0_IN",
    DEV_EP1_IN: "EP1_IN",
    DEV_EP1_ACK: "EP1_ACK",
    DEV_LOG: "LOG",
}

#: Default bulk chunk size for the guest EP1 OUT path.  LK's fastboot download
#: loop consumes one 16 KiB bulk transfer per QMU GPD; larger frames would be
#: written past the 16 KiB guest buffer by the current model.
EP1_CHUNK_SIZE = 16384

# ---------------------------------------------------------------------------
# USB constants (only what the virtual host needs)
# ---------------------------------------------------------------------------
USB_DIR_IN = 0x80
USB_REQ_GET_DESCRIPTOR = 0x06
USB_REQ_SET_ADDRESS = 0x05
USB_REQ_SET_CONFIGURATION = 0x09

USB_DT_DEVICE = 0x01
USB_DT_CONFIG = 0x02
USB_DT_STRING = 0x03
USB_DT_INTERFACE = 0x04
USB_DT_ENDPOINT = 0x05

EP_TYPE_CONTROL = 0
EP_TYPE_ISOCHRONOUS = 1
EP_TYPE_BULK = 2
EP_TYPE_INTERRUPT = 3


# ---------------------------------------------------------------------------
# errors
# ---------------------------------------------------------------------------
class VusbError(Exception):
    """Base error for the virtual-USB host library."""


class VusbConnectionError(VusbError):
    """The chardev socket could not be opened or was closed."""


class VusbTimeout(VusbError):
    """A frame / log line did not arrive within the requested time."""


class ProtocolError(VusbError):
    """The peer sent something that does not fit the protocol."""


# ---------------------------------------------------------------------------
# frame primitives
# ---------------------------------------------------------------------------
@dataclasses.dataclass(frozen=True)
class Frame:
    """One decoded chardev frame."""

    type: int
    payload: bytes = b""

    def __post_init__(self) -> None:
        if not 0 <= int(self.type) <= 0xFF:
            raise ValueError("frame type out of range: %r" % (self.type,))
        if len(self.payload) > MAX_FRAME_PAYLOAD:
            raise ValueError(
                "frame payload too large (%d > %d)" % (len(self.payload), MAX_FRAME_PAYLOAD)
            )

    @property
    def name(self) -> str:
        return FRAME_NAMES.get(self.type, "0x%02x" % self.type)

    def __repr__(self) -> str:  # keep debug output compact
        preview = self.payload[:16].hex()
        if len(self.payload) > 16:
            preview += "..."
        return "Frame(%s len=%d %s)" % (self.name, len(self.payload), preview)


def encode_frame(frame_type: int, payload: bytes = b"") -> bytes:
    """Encode one chardev frame (u8 type + u16le len + payload)."""
    frame_type = int(frame_type)
    if not 0 <= frame_type <= 0xFF:
        raise ValueError("frame type out of range: %r" % (frame_type,))
    payload = bytes(payload)
    if len(payload) > MAX_FRAME_PAYLOAD:
        raise ValueError(
            "frame payload too large (%d > %d)" % (len(payload), MAX_FRAME_PAYLOAD)
        )
    return FRAME_HEADER.pack(frame_type, len(payload)) + payload


class FrameDecoder:
    """Incremental decoder: feed() arbitrary TCP chunks, get complete frames."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def reset(self) -> None:
        self._buffer.clear()

    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)

    def feed(self, data: bytes) -> List[Frame]:
        self._buffer.extend(data)
        frames: List[Frame] = []
        while len(self._buffer) >= FRAME_HEADER.size:
            frame_type = self._buffer[0]
            payload_len = self._buffer[1] | (self._buffer[2] << 8)
            end = FRAME_HEADER.size + payload_len
            if len(self._buffer) < end:
                break
            payload = bytes(self._buffer[FRAME_HEADER.size:end])
            del self._buffer[:end]
            frames.append(Frame(frame_type, payload))
        return frames


# ---------------------------------------------------------------------------
# USB descriptors
# ---------------------------------------------------------------------------
def _u16(data: bytes, offset: int) -> int:
    return data[offset] | (data[offset + 1] << 8)


@dataclasses.dataclass(frozen=True)
class EndpointDescriptor:
    bLength: int
    bDescriptorType: int
    bEndpointAddress: int
    bmAttributes: int
    wMaxPacketSize: int
    bInterval: int

    @property
    def number(self) -> int:
        return self.bEndpointAddress & 0x0F

    @property
    def direction_in(self) -> bool:
        return bool(self.bEndpointAddress & USB_DIR_IN)

    @property
    def transfer_type(self) -> int:
        return self.bmAttributes & 0x03

    @classmethod
    def parse(cls, data: bytes) -> "EndpointDescriptor":
        if len(data) < 7 or data[0] < 7:
            raise ProtocolError("truncated endpoint descriptor: %s" % bytes(data).hex())
        return cls(
            bLength=data[0],
            bDescriptorType=data[1],
            bEndpointAddress=data[2],
            bmAttributes=data[3],
            wMaxPacketSize=_u16(data, 4),
            bInterval=data[6],
        )


@dataclasses.dataclass
class InterfaceDescriptor:
    bLength: int
    bDescriptorType: int
    bInterfaceNumber: int
    bAlternateSetting: int
    bNumEndpoints: int
    bInterfaceClass: int
    bInterfaceSubClass: int
    bInterfaceProtocol: int
    iInterface: int
    endpoints: List[EndpointDescriptor] = dataclasses.field(default_factory=list)

    @classmethod
    def parse(cls, data: bytes) -> "InterfaceDescriptor":
        if len(data) < 9 or data[0] < 9:
            raise ProtocolError("truncated interface descriptor: %s" % bytes(data).hex())
        return cls(
            bLength=data[0],
            bDescriptorType=data[1],
            bInterfaceNumber=data[2],
            bAlternateSetting=data[3],
            bNumEndpoints=data[4],
            bInterfaceClass=data[5],
            bInterfaceSubClass=data[6],
            bInterfaceProtocol=data[7],
            iInterface=data[8],
        )


@dataclasses.dataclass(frozen=True)
class DeviceDescriptor:
    bLength: int
    bDescriptorType: int
    bcdUSB: int
    bDeviceClass: int
    bDeviceSubClass: int
    bDeviceProtocol: int
    bMaxPacketSize0: int
    idVendor: int
    idProduct: int
    bcdDevice: int
    iManufacturer: int
    iProduct: int
    iSerialNumber: int
    bNumConfigurations: int

    @classmethod
    def parse(cls, data: bytes) -> "DeviceDescriptor":
        data = bytes(data)
        if len(data) < 18:
            raise ProtocolError("truncated device descriptor (%d bytes): %s" % (len(data), data.hex()))
        return cls(
            bLength=data[0],
            bDescriptorType=data[1],
            bcdUSB=_u16(data, 2),
            bDeviceClass=data[4],
            bDeviceSubClass=data[5],
            bDeviceProtocol=data[6],
            bMaxPacketSize0=data[7],
            idVendor=_u16(data, 8),
            idProduct=_u16(data, 10),
            bcdDevice=_u16(data, 12),
            iManufacturer=data[14],
            iProduct=data[15],
            iSerialNumber=data[16],
            bNumConfigurations=data[17],
        )

    def raw(self) -> bytes:
        return struct.pack(
            "<BBHBBBBHHHBBBB",
            self.bLength,
            self.bDescriptorType,
            self.bcdUSB,
            self.bDeviceClass,
            self.bDeviceSubClass,
            self.bDeviceProtocol,
            self.bMaxPacketSize0,
            self.idVendor,
            self.idProduct,
            self.bcdDevice,
            self.iManufacturer,
            self.iProduct,
            self.iSerialNumber,
            self.bNumConfigurations,
        )


@dataclasses.dataclass
class ConfigDescriptor:
    raw: bytes
    wTotalLength: int
    bNumInterfaces: int
    bConfigurationValue: int
    iConfiguration: int
    bmAttributes: int
    bMaxPower: int
    interfaces: List[InterfaceDescriptor] = dataclasses.field(default_factory=list)

    @property
    def endpoints(self) -> List[EndpointDescriptor]:
        result: List[EndpointDescriptor] = []
        for interface in self.interfaces:
            result.extend(interface.endpoints)
        return result

    def find_bulk_endpoints(self) -> Dict[str, Optional[EndpointDescriptor]]:
        """Return {'in': endpoint, 'out': endpoint} for the first bulk pair."""
        found: Dict[str, Optional[EndpointDescriptor]] = {"in": None, "out": None}
        for ep in self.endpoints:
            if ep.transfer_type != EP_TYPE_BULK:
                continue
            if ep.direction_in and found["in"] is None:
                found["in"] = ep
            elif not ep.direction_in and found["out"] is None:
                found["out"] = ep
        return found

    @classmethod
    def parse(cls, data: bytes) -> "ConfigDescriptor":
        data = bytes(data)
        if len(data) < 9:
            raise ProtocolError("truncated config descriptor (%d bytes): %s" % (len(data), data.hex()))
        total = _u16(data, 2)
        if total < 9:
            raise ProtocolError("config descriptor wTotalLength=%d looks wrong" % total)
        raw = data[:total] if total <= len(data) else data
        config = cls(
            raw=raw,
            wTotalLength=total,
            bNumInterfaces=data[4],
            bConfigurationValue=data[5],
            iConfiguration=data[6],
            bmAttributes=data[7],
            bMaxPower=data[8],
        )
        offset = 0
        current_interface: Optional[InterfaceDescriptor] = None
        while offset + 2 <= len(raw):
            length = raw[offset]
            desc_type = raw[offset + 1]
            if length < 2:
                raise ProtocolError("bad descriptor length %d at offset %d" % (length, offset))
            chunk = raw[offset:offset + length]
            if desc_type == USB_DT_INTERFACE and length >= 9:
                current_interface = InterfaceDescriptor.parse(chunk)
                config.interfaces.append(current_interface)
            elif desc_type == USB_DT_ENDPOINT and length >= 7:
                ep = EndpointDescriptor.parse(chunk)
                if current_interface is not None:
                    current_interface.endpoints.append(ep)
                else:
                    raise ProtocolError("endpoint descriptor before interface descriptor")
            offset += length
        return config


@dataclasses.dataclass
class UsbDeviceInfo:
    device: DeviceDescriptor
    config: Optional[ConfigDescriptor]
    strings: Dict[int, str] = dataclasses.field(default_factory=dict)
    address: int = 1

    @property
    def vendor_id(self) -> int:
        return self.device.idVendor

    @property
    def product_id(self) -> int:
        return self.device.idProduct

    @property
    def bulk_in(self) -> Optional[EndpointDescriptor]:
        if self.config is None:
            return None
        return self.config.find_bulk_endpoints()["in"]

    @property
    def bulk_out(self) -> Optional[EndpointDescriptor]:
        if self.config is None:
            return None
        return self.config.find_bulk_endpoints()["out"]

    def summary(self) -> str:
        return "VID:PID=%04x:%04x cfg=%s" % (
            self.device.idVendor,
            self.device.idProduct,
            "-" if self.config is None else self.config.bConfigurationValue,
        )


def parse_string_descriptor(data: bytes, index: int = 0) -> str:
    """Decode a USB string descriptor (bLength/bDescriptorType/UTF-16LE)."""
    data = bytes(data)
    if len(data) < 2 or data[1] != USB_DT_STRING:
        raise ProtocolError("not a string descriptor: %s" % data.hex())
    length = min(data[0], len(data))
    payload = data[2:length]
    if index == 0:
        langids = [_u16(payload, off) for off in range(0, len(payload) - 1, 2)]
        return ",".join("0x%04x" % lang for lang in langids)
    return payload.decode("utf-16-le", "replace")


@dataclasses.dataclass(frozen=True)
class TransferStats:
    """Result of one bulk transfer."""

    byte_count: int
    chunk_count: int
    seconds: float
    acks: int = 0

    @property
    def bytes_per_second(self) -> float:
        if self.seconds <= 0:
            return 0.0
        return self.byte_count / self.seconds

    def __str__(self) -> str:  # pragma: no cover - cosmetic
        return "%d bytes in %.3fs (%.1f MiB/s)" % (
            self.byte_count,
            self.seconds,
            self.bytes_per_second / (1024.0 * 1024.0),
        )


@dataclasses.dataclass(frozen=True)
class AckRecord:
    """One EP1 RX-completion ACK frame received from the device model."""

    index: int
    payload: bytes
    at: float

    def counter(self) -> Optional[int]:
        """Best-effort decode of a u32le counter/byte-count payload (if any)."""
        if len(self.payload) >= 4:
            return int.from_bytes(self.payload[:4], "little")
        return None

    def __str__(self) -> str:
        counter = self.counter()
        prefix = "#%d" % self.index
        if counter is not None:
            prefix += " counter=%d" % counter
        return "%s payload=%s @%.3fs" % (prefix, self.payload.hex(), self.at)


ProgressCallback = Callable[[int, int, float], None]


# ---------------------------------------------------------------------------
# the host itself
# ---------------------------------------------------------------------------
class VUsbHost:
    """TCP client speaking the virtual-UDC framed chardev protocol.

    Parameters
    ----------
    host, port:
        QEMU -chardev socket address (MTK_USBDEV_TCP, default 4444).
    connect_timeout:
        How long to keep retrying the TCP connect (QEMU may still boot).
    io_timeout:
        Default timeout for a single control/bulk response.
    record_logs:
        Keep every 0x90 LOG line in :attr:'logs' (needed to wait for
        SOFT_CONN / QMU_READY).
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 4444,
        *,
        connect_timeout: float = 30.0,
        io_timeout: float = 10.0,
        record_logs: bool = True,
        ack_frame_type: int = DEV_EP1_ACK,
        record_traffic: bool = False,
    ) -> None:
        self.host = str(host)
        self.port = int(port)
        self.connect_timeout = float(connect_timeout)
        self.io_timeout = float(io_timeout)
        self.record_logs = bool(record_logs)

        self._sock: Optional[socket.socket] = None
        self._decoder = FrameDecoder()
        self._frames: Deque[Frame] = deque()
        self.logs: List[str] = []
        self.tx_frames = 0
        self.rx_frames = 0
        self.ack_frame_type = int(ack_frame_type)
        self.record_traffic = bool(record_traffic)
        self._t0 = time.monotonic()
        self.acks: List[AckRecord] = []
        self._traffic: List[str] = []

    # -- connection management ---------------------------------------------
    def connect(self, timeout: Optional[float] = None) -> "VUsbHost":
        """Open the chardev socket, retrying until *timeout* elapses."""
        if self._sock is not None:
            return self
        deadline = time.monotonic() + (self.connect_timeout if timeout is None else float(timeout))
        last_error: Optional[BaseException] = None
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise VusbConnectionError(
                    "cannot connect to %s:%d within %.1fs: %s"
                    % (self.host, self.port, self.connect_timeout, last_error)
                )
            try:
                sock = socket.create_connection(
                    (self.host, self.port), timeout=min(remaining, 2.0)
                )
                break
            except OSError as exc:
                last_error = exc
                time.sleep(min(0.25, max(0.0, remaining)))
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:  # pragma: no cover - platform dependent
            pass
        self._sock = sock
        self._decoder.reset()
        self._frames.clear()
        self._t0 = time.monotonic()
        LOG.info("connected to chardev %s:%d", self.host, self.port)
        return self

    def close(self, *, send_disconnect: bool = False) -> None:
        """Close the chardev socket (optionally telling the model first)."""
        if self._sock is None:
            return
        if send_disconnect:
            try:
                self.send_frame(HOST_DISCONNECT)
            except VusbError:
                pass
        sock, self._sock = self._sock, None
        try:
            sock.close()
        except OSError:  # pragma: no cover
            pass
        LOG.info("chardev closed")

    def reconnect(
        self,
        *,
        attempts: int = 5,
        delay: float = 0.3,
        connect_timeout: Optional[float] = None,
    ) -> "VUsbHost":
        """Drop the current socket and re-open it (chardev server keeps running)."""
        self.close()
        last_error: Optional[BaseException] = None
        for attempt in range(max(1, attempts)):
            try:
                return self.connect(timeout=connect_timeout)
            except VusbConnectionError as exc:
                last_error = exc
                LOG.warning("reconnect attempt %d failed: %s", attempt + 1, exc)
                time.sleep(delay)
        raise VusbConnectionError("reconnect failed: %s" % (last_error,))

    @property
    def connected(self) -> bool:
        return self._sock is not None

    def __enter__(self) -> "VUsbHost":
        return self.connect()

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # -- low level I/O ------------------------------------------------------
    def _require_socket(self) -> socket.socket:
        if self._sock is None:
            raise VusbConnectionError("chardev is not connected (call connect() first)")
        return self._sock

    def _pump(self, timeout: float) -> int:
        """Wait up to *timeout* for data and decode complete frames.

        Returns the number of decoded frames.  LOG frames are stored in
        :attr:'logs', every other frame is queued for :meth:'recv_frame'.
        """
        sock = self._require_socket()
        try:
            readable, _, _ = select.select([sock], [], [], max(0.0, timeout))
        except (OSError, ValueError) as exc:  # pragma: no cover - socket torn down
            raise VusbConnectionError("chardev select failed: %s" % (exc,)) from exc
        if not readable:
            return 0
        try:
            data = sock.recv(65536)
        except OSError as exc:
            raise VusbConnectionError("chardev recv failed: %s" % (exc,)) from exc
        if not data:
            raise VusbConnectionError("chardev closed by peer")
        frames = self._decoder.feed(data)
        for frame in frames:
            self.rx_frames += 1
            if frame.type == DEV_LOG:
                self._record_log(frame.payload)
            elif frame.type == self.ack_frame_type:
                self._record_ack(frame.payload)
            else:
                self._frames.append(frame)
                LOG.debug("RX %r", frame)
        return len(frames)

    def _record_log(self, payload: bytes) -> None:
        text = payload.decode("latin-1").rstrip("\x00")
        if self.record_logs:
            self.logs.append(text)
        self._record_traffic("RX", DEV_LOG, payload)
        LOG.debug("DEV LOG: %s", text)

    def _record_ack(self, payload: bytes) -> None:
        ack = AckRecord(len(self.acks), bytes(payload), time.monotonic() - self._t0)
        self.acks.append(ack)
        self._record_traffic("RX", self.ack_frame_type, payload, note=str(ack))
        LOG.debug("EP1 ACK %s", ack)

    @property
    def ack_count(self) -> int:
        return len(self.acks)

    @property
    def last_ack(self) -> Optional[AckRecord]:
        return self.acks[-1] if self.acks else None

    def wait_for_acks(self, target_count: int, timeout: Optional[float] = None) -> int:
        """Block until *target_count* EP1 ACK frames were received in total.

        Raises :class:'VusbTimeout' when the device model does not ACK in time
        (for example a model build without the ACK frame).
        """
        limit = self.io_timeout if timeout is None else float(timeout)
        deadline = time.monotonic() + limit
        target = int(target_count)
        while self.ack_count < target:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise VusbTimeout(
                    "timed out after %.2fs waiting for EP1 ACK #%d "
                    "(received %d; expected frame type 0x%02x)"
                    % (limit, target, self.ack_count, self.ack_frame_type)
                )
            self._pump(min(remaining, 0.05))
        return self.ack_count

    def record_event(self, text: str) -> None:
        """Append a host-side event line to the traffic dump (diagnostics)."""
        self._record_traffic("EVENT", 0, b"", note=str(text))

    def _record_traffic(self, direction: str, frame_type: int, payload: bytes,
                        note: str = "") -> None:
        if not self.record_traffic:
            return
        elapsed = time.monotonic() - self._t0
        if frame_type == DEV_LOG:
            preview = payload.decode("latin-1").rstrip("\x00").replace("\n", " ")
        elif frame_type in (DEV_EP0_IN, DEV_EP1_IN):
            # Fastboot/descriptor replies are mostly printable ASCII: keep them
            # readable in the dump instead of hex when possible.
            sample = payload[:48]
            if sample and all(32 <= byte < 127 or byte in (9, 10, 13) for byte in sample):
                preview = repr(sample.decode("latin-1"))
            else:
                preview = sample.hex()
            if len(payload) > len(sample):
                preview += "..."
        elif frame_type:
            preview = payload[:48].hex()
            if len(payload) > 48:
                preview += "..."
        else:
            preview = ""
        name = FRAME_NAMES.get(frame_type, "0x%02x" % frame_type) if frame_type else "EVENT"
        line = "%9.3fs %-5s %-12s len=%-6d %s" % (
            elapsed, direction, name, len(payload), preview
        )
        if note:
            line += "  # " + note
        self._traffic.append(line)

    def dump_traffic(self, path: str) -> str:
        """Write the recorded frame/event/LOG traffic log to *path*."""
        lines = [
            "# vusb traffic dump host=%s port=%d connected=%s tx_frames=%d "
            "rx_frames=%d acks=%d" % (
                self.host, self.port, self.connected, self.tx_frames,
                self.rx_frames, self.ack_count,
            ),
        ]
        lines.extend(self._traffic)
        lines.append("")
        lines.append("# LOG lines")
        lines.extend(self.logs)
        with open(path, "w", encoding="utf-8") as handle:
            handle.write("\n".join(lines) + "\n")
        return path

    def send_frame(self, frame_type: int, payload: bytes = b"") -> None:
        """Send one framed message to the device model."""
        sock = self._require_socket()
        data = encode_frame(frame_type, payload)
        try:
            sock.sendall(data)
        except OSError as exc:
            raise VusbConnectionError("chardev send failed: %s" % (exc,)) from exc
        self.tx_frames += 1
        self._record_traffic("TX", int(frame_type), bytes(payload))
        LOG.debug("TX %r", Frame(int(frame_type), bytes(payload)))

    def recv_frame(self, timeout: Optional[float] = None) -> Frame:
        """Return the next non-LOG frame or raise :class:'VusbTimeout'."""
        limit = self.io_timeout if timeout is None else float(timeout)
        deadline = time.monotonic() + limit
        while not self._frames:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise VusbTimeout("timed out after %.2fs waiting for a chardev frame" % limit)
            self._pump(remaining)
        return self._frames.popleft()

    def try_recv_frame(self) -> Optional[Frame]:
        """Non-blocking variant of :meth:'recv_frame' (None if nothing queued)."""
        if not self._frames:
            self._pump(0.0)
        return self._frames.popleft() if self._frames else None

    def pump(self, seconds: float) -> int:
        """Keep reading frames (into the queue / log history) for *seconds*."""
        deadline = time.monotonic() + max(0.0, float(seconds))
        count = 0
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return count
            count += self._pump(remaining)

    # -- log helpers --------------------------------------------------------
    def log_count(self, needle: str) -> int:
        return sum(1 for line in self.logs if needle in line)

    def wait_log(self, needle: str, timeout: float = 10.0) -> bool:
        """Wait until a LOG line containing *needle* was seen (history too)."""
        if self.log_count(needle):
            return True
        deadline = time.monotonic() + max(0.0, float(timeout))
        while True:
            if self.log_count(needle):
                return True
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            self._pump(min(0.05, remaining))

    def wait_ready(self, timeout: float = 5.0) -> bool:
        """Wait for the model's LOG "READY" (sent when the chardev opens)."""
        return self.wait_log("READY", timeout)

    def wait_pullup(
        self,
        timeout: float = 150.0,
        settle: float = 0.5,
        max_reinit_rounds: int = 6,
    ) -> int:
        """Wait for LOG "SOFT_CONN" and let a possible UDC re-init settle.

        LK raises the U2 pull-up (POWER_MANAGEMENT.SOFT_CONN) once its UDC
        driver is initialised; the model logs SOFT_CONN.  Sometimes the guest
        re-initialises the UDC once more, so after the first hit we keep
        pumping until no new SOFT_CONN line appears for *settle* seconds.
        Returns the final SOFT_CONN count.
        """
        if not self.wait_log("SOFT_CONN", timeout):
            raise VusbTimeout(
                "timed out after %.1fs waiting for LOG SOFT_CONN (U2 pull-up); "
                "is LK in fastboot? (MTK_RGU_KEY=3)"
                % timeout
            )
        for _ in range(max_reinit_rounds):
            count = self.log_count("SOFT_CONN")
            self.pump(settle)
            after = self.log_count("SOFT_CONN")
            if after == count:
                break
            LOG.info("U2 pull-up re-asserted (%d -> %d), settling again", count, after)
        total = self.log_count("SOFT_CONN")
        LOG.info("SOFT_CONN seen %d time(s)", total)
        return total

    # -- bus / control transfers -------------------------------------------
    def connect_bus(self, wait_connected: float = 1.0) -> bool:
        """Send HOST_CONNECT (bus reset) and wait for the model's CONNECTED log."""
        self.send_frame(HOST_CONNECT)
        LOG.debug("-> HOST_CONNECT (bus reset)")
        return self.wait_log("CONNECTED", wait_connected)

    def disconnect_bus(self) -> None:
        self.send_frame(HOST_DISCONNECT)

    def _send_setup(
        self,
        bm_request_type: int,
        b_request: int,
        w_value: int = 0,
        w_index: int = 0,
        w_length: int = 0,
    ) -> None:
        packet = struct.pack("<BBHHH", bm_request_type & 0xFF, b_request & 0xFF,
                             w_value & 0xFFFF, w_index & 0xFFFF, w_length & 0xFFFF)
        LOG.debug("SETUP bm=%02x req=%02x val=%04x idx=%04x len=%d",
                  bm_request_type, b_request, w_value, w_index, w_length)
        self.send_frame(HOST_SETUP, packet)

    def control_out_no_data(
        self,
        bm_request_type: int,
        b_request: int,
        w_value: int = 0,
        w_index: int = 0,
        *,
        settle: float = 0.0,
    ) -> None:
        """Send a no-data control OUT request.

        The model has no completion frame for control-OUT transfers, so the
        caller only gets an optional settle delay.
        """
        self._send_setup(bm_request_type, b_request, w_value, w_index, 0)
        if settle > 0:
            self.pump(settle)

    def control_in(
        self,
        bm_request_type: int,
        b_request: int,
        w_value: int = 0,
        w_index: int = 0,
        w_length: int = 0,
        *,
        timeout: Optional[float] = None,
        idle_gap: float = 0.05,
    ) -> bytes:
        """Perform a control IN transfer and return the received bytes.

        The model emits one EP0 IN frame per guest EP0CSR.TXPKTRDY write, so
        frames are accumulated until *w_length* bytes arrived or the guest
        stops sending (short descriptor) for *idle_gap* seconds.
        """
        self._send_setup(bm_request_type, b_request, w_value, w_index, w_length)
        deadline = time.monotonic() + (self.io_timeout if timeout is None else float(timeout))
        chunks: List[bytes] = []
        try:
            frame = self.recv_frame(max(0.0, deadline - time.monotonic()))
        except VusbTimeout as exc:
            raise VusbTimeout(
                "no EP0 IN data for control request 0x%02x (wLength=%d)"
                % (b_request, w_length)
            ) from exc
        if frame.type != DEV_EP0_IN:
            self._frames.appendleft(frame)
            raise ProtocolError(
                "expected EP0 IN frame, got %s while handling request 0x%02x"
                % (frame.name, b_request)
            )
        chunks.append(frame.payload)
        received = len(frame.payload)
        while received < w_length:
            try:
                frame = self.recv_frame(idle_gap)
            except VusbTimeout:
                break
            if frame.type != DEV_EP0_IN:
                self._frames.appendleft(frame)
                break
            chunks.append(frame.payload)
            received += len(frame.payload)
        # The model treats EP0 ACK as a no-op, but send it for protocol fidelity.
        try:
            self.send_frame(HOST_EP0_ACK)
        except VusbConnectionError:
            pass
        return b"".join(chunks)

    def get_descriptor(
        self,
        descriptor_type: int,
        index: int = 0,
        lang_id: int = 0,
        length: Optional[int] = None,
        *,
        timeout: Optional[float] = None,
    ) -> bytes:
        """Standard GET_DESCRIPTOR control request."""
        if length is None:
            length = 18 if descriptor_type == USB_DT_DEVICE else 255
        return self.control_in(
            USB_DIR_IN,
            USB_REQ_GET_DESCRIPTOR,
            (descriptor_type << 8) | (index & 0xFF),
            lang_id,
            length,
            timeout=timeout,
        )

    def get_string(self, index: int, lang_id: int = 0x0409) -> str:
        raw = self.get_descriptor(USB_DT_STRING, index & 0xFF, lang_id, 255)
        return parse_string_descriptor(raw, index)

    def set_address(self, address: int) -> None:
        self.control_out_no_data(0x00, USB_REQ_SET_ADDRESS, address & 0x7F, 0)

    def set_configuration(self, configuration: int = 1) -> None:
        self.control_out_no_data(0x00, USB_REQ_SET_CONFIGURATION, configuration & 0xFF, 0)

    def enumerate(
        self,
        *,
        address: int = 1,
        configure: bool = True,
        include_strings: bool = False,
        descriptor_settle: float = 0.25,
    ) -> UsbDeviceInfo:
        """Run the standard fastboot-device enumeration sequence.

        GET_DESCRIPTOR(device) -> SET_ADDRESS -> GET_DESCRIPTOR(config) ->
        SET_CONFIGURATION.  String descriptors are only fetched when
        *include_strings* is true (LK advertises string indexes but does not
        answer string requests on the fastboot interface).
        """
        device = DeviceDescriptor.parse(self.get_descriptor(USB_DT_DEVICE, length=64))
        LOG.info("device descriptor: %04x:%04x bcdUSB=%04x",
                 device.idVendor, device.idProduct, device.bcdUSB)
        self.set_address(address)
        if descriptor_settle > 0:
            self.pump(descriptor_settle)
        config = ConfigDescriptor.parse(self.get_descriptor(USB_DT_CONFIG, length=255))
        LOG.info("config descriptor: %d interface(s), %d endpoint(s), %d bytes",
                 len(config.interfaces), len(config.endpoints), len(config.raw))
        strings: Dict[int, str] = {}
        if include_strings:
            for index in (device.iManufacturer, device.iProduct, device.iSerialNumber):
                if not index:
                    continue
                try:
                    strings[index] = self.get_string(index)
                except (VusbTimeout, ProtocolError) as exc:
                    LOG.warning("string descriptor %d not available: %s", index, exc)
        info = UsbDeviceInfo(device=device, config=config, strings=strings, address=address)
        if configure:
            self.set_configuration(config.bConfigurationValue or 1)
            LOG.info("SET_CONFIGURATION(%d) sent", config.bConfigurationValue or 1)
        return info

    # -- bulk transfers -----------------------------------------------------
    def bulk_out_raw(self, payload: bytes) -> None:
        """Send one EP1 OUT frame (fastboot command bytes)."""
        self.send_frame(HOST_EP1_OUT, bytes(payload))

    def bulk_out(
        self,
        data: bytes,
        *,
        chunk_size: int = EP1_CHUNK_SIZE,
        chunk_delay: float = 0.0,
        progress: Optional[ProgressCallback] = None,
    ) -> TransferStats:
        """Send *data* over EP1 as fixed-size bulk OUT frames.

        *chunk_size* must stay at 16 KiB for the current model: each frame is
        copied into one 16 KiB guest QMU buffer.  *chunk_delay* adds a sleep
        between chunks (flow-control safety valve for the real model, where a
        chunk is silently dropped if the guest has not re-armed a GPD yet).
        """
        data = bytes(data)
        if chunk_size <= 0 or chunk_size > MAX_FRAME_PAYLOAD:
            raise ValueError("chunk_size must be in 1..%d" % MAX_FRAME_PAYLOAD)
        total = len(data)
        started = time.monotonic()
        sent = 0
        chunks = 0
        for offset in range(0, total, chunk_size):
            chunk = data[offset:offset + chunk_size]
            self.send_frame(HOST_EP1_OUT, chunk)
            sent += len(chunk)
            chunks += 1
            if progress is not None:
                progress(sent, total, time.monotonic() - started)
            if chunk_delay > 0 and sent < total:
                time.sleep(chunk_delay)
        stats = TransferStats(byte_count=sent, chunk_count=chunks,
                              seconds=time.monotonic() - started)
        LOG.debug("bulk OUT %s", stats)
        return stats

    def bulk_out_paced(
        self,
        data: bytes,
        *,
        window: int = 4,
        ack_timeout: Optional[float] = None,
        chunk_size: int = EP1_CHUNK_SIZE,
        progress: Optional[ProgressCallback] = None,
    ) -> TransferStats:
        """Send data over EP1 with ACK credit flow control.

        The QEMU UDC model is expected to send one 0x83 ACK frame per completed
        guest RX GPD (16 KiB chunk).  At most *window* chunks may be in flight:
        a new chunk is only sent after enough ACK frames arrived.  This makes
        the transfer self-clocking and avoids the silent RX-GPD drops of the
        old timing-based pacing.
        """
        data = bytes(data)
        if chunk_size <= 0 or chunk_size > MAX_FRAME_PAYLOAD:
            raise ValueError("chunk_size must be in 1..%d" % MAX_FRAME_PAYLOAD)
        if window < 1:
            raise ValueError("window must be >= 1")
        total = len(data)
        started = time.monotonic()
        base = self.ack_count
        limit = self.io_timeout if ack_timeout is None else float(ack_timeout)
        sent = 0
        chunks = 0
        for offset in range(0, total, chunk_size):
            while chunks - (self.ack_count - base) >= window:
                target = base + chunks - window + 1
                self.wait_for_acks(target, timeout=limit)
            chunk = data[offset:offset + chunk_size]
            self.send_frame(HOST_EP1_OUT, chunk)
            sent += len(chunk)
            chunks += 1
            if progress is not None:
                progress(sent, total, time.monotonic() - started)
        # Confirm the model consumed every chunk (also detects a model build
        # without ACK support even when the image fits in one window).
        if chunks:
            self.wait_for_acks(base + chunks, timeout=limit)
        stats = TransferStats(
            byte_count=sent,
            chunk_count=chunks,
            seconds=time.monotonic() - started,
            acks=self.ack_count - base,
        )
        LOG.debug("bulk OUT (ack paced) %s", stats)
        return stats

    def bulk_in_frames(
        self,
        *,
        timeout: Optional[float] = None,
        idle_gap: float = 0.05,
        first_timeout: Optional[float] = None,
    ) -> List[bytes]:
        """Receive EP1 IN data as the list of individual frames.

        The model sends one 0x82 frame per completed TX GPD, so a single
        fastboot reply can arrive in several frames.  Keeping the frame
        boundaries lets the fastboot client split interleaved INFO messages
        from the terminal OKAY/FAIL status.
        """
        limit = self.io_timeout if timeout is None else float(timeout)
        deadline = time.monotonic() + limit
        wait_first = limit if first_timeout is None else float(first_timeout)
        try:
            frame = self.recv_frame(wait_first)
        except VusbTimeout as exc:
            raise VusbTimeout("no EP1 IN data within %.1fs" % limit) from exc
        if frame.type != DEV_EP1_IN:
            self._frames.appendleft(frame)
            raise ProtocolError("expected EP1 IN frame, got %s" % frame.name)
        chunks = [frame.payload]
        while time.monotonic() < deadline:
            try:
                frame = self.recv_frame(idle_gap)
            except VusbTimeout:
                break
            except VusbConnectionError as exc:
                # The guest may reset right after the final reply (reboot);
                # keep what we already received.
                LOG.info("connection closed while draining EP1 IN: %s", exc)
                break
            if frame.type != DEV_EP1_IN:
                self._frames.appendleft(frame)
                break
            chunks.append(frame.payload)
        LOG.debug("bulk IN %d frame(s), %d bytes", len(chunks),
                  sum(len(chunk) for chunk in chunks))
        return chunks

    def bulk_in(
        self,
        *,
        timeout: Optional[float] = None,
        idle_gap: float = 0.05,
        first_timeout: Optional[float] = None,
    ) -> bytes:
        """Receive EP1 IN data, concatenating frames until the peer goes idle."""
        return b"".join(
            self.bulk_in_frames(timeout=timeout, idle_gap=idle_gap,
                                first_timeout=first_timeout)
        )

    def __repr__(self) -> str:  # pragma: no cover - cosmetic
        return "VUsbHost(%s:%d, connected=%s)" % (self.host, self.port, self.connected)
