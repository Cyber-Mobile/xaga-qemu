"""Pure-Python fake mtu3 UDC for host-side tests (no QEMU required).

MockChardevServer listens on a TCP socket and speaks exactly the framed
protocol implemented by hw/misc/mtk_usbdev.c, so the real host library
(lib/vusb.py, lib/fastboot.py) can be regression-tested without booting LK.

Two virtual devices are provided:

* MockFastbootDevice -- LK fastboot gadget: descriptors taken from a real LK
  run (VID:PID 18d1:d00d), LOG lines READY / SOFT_CONN / CONNECTED /
  QMU_READY, EP0 enumeration, and the fastboot command protocol over EP1
  (getvar, download, flash, reboot, oem, erase).
* MockAdbDevice -- adb (USB ffs) gadget skeleton used by lib/adb.py tests:
  answers the CNXN handshake and the basic OPEN/WRTE/CLOSE packets.

Example
-------
    server = MockChardevServer(MockFastbootDevice).start()
    host = vusb.VUsbHost("127.0.0.1", server.port)
    host.connect()
    ...
    server.stop()

Only the Python standard library is used.
"""
from __future__ import annotations

import logging
import socket
import struct
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple

from . import vusb

LOG = logging.getLogger("vusb.mock")

# ---------------------------------------------------------------------------
# descriptor helpers
# ---------------------------------------------------------------------------
def build_device_descriptor(
    vendor_id: int = 0x18D1,
    product_id: int = 0xD00D,
    bcd_usb: int = 0x0200,
    bcd_device: int = 0x0100,
    max_packet0: int = 64,
) -> bytes:
    """18-byte USB device descriptor (same layout as the real LK gadget)."""
    return struct.pack(
        "<BBHBBBBHHHBBBB",
        18,             # bLength
        vusb.USB_DT_DEVICE,
        bcd_usb,
        0, 0, 0,        # bDeviceClass / SubClass / Protocol
        max_packet0,
        vendor_id,
        product_id,
        bcd_device,
        1, 2, 3,        # iManufacturer / iProduct / iSerialNumber
        1,              # bNumConfigurations
    )


def build_config_descriptor(
    interface_class: int = 0xFF,
    interface_subclass: int = 0x42,
    interface_protocol: int = 0x03,
    in_endpoint: int = 0x81,
    out_endpoint: int = 0x01,
    max_packet: int = 512,
    i_interface: int = 4,
) -> bytes:
    """9 + 9 + 7 + 7 byte config descriptor with one bulk-IN/bulk-OUT pair."""
    interface = struct.pack(
        "<BBBBBBBBB",
        9, vusb.USB_DT_INTERFACE, 0, 0, 2,
        interface_class, interface_subclass, interface_protocol, i_interface,
    )
    ep_in = struct.pack("<BBBBHB", 7, vusb.USB_DT_ENDPOINT, in_endpoint, 2, max_packet, 1)
    ep_out = struct.pack("<BBBBHB", 7, vusb.USB_DT_ENDPOINT, out_endpoint, 2, max_packet, 0)
    body = interface + ep_in + ep_out
    header = struct.pack("<BBHBBBBB", 9, vusb.USB_DT_CONFIG, 9 + len(body), 1, 1, 0, 0x80, 0x80)
    return header + body


def build_string_descriptor(text: str) -> bytes:
    payload = text.encode("utf-16-le")
    return bytes([2 + len(payload), vusb.USB_DT_STRING]) + payload


def wait_until(predicate: Callable[[], bool], timeout: float = 5.0, interval: float = 0.005) -> bool:
    """Poll *predicate* until true or *timeout* elapses."""
    deadline = time.monotonic() + max(0.0, float(timeout))
    while True:
        if predicate():
            return True
        if time.monotonic() >= deadline:
            return False
        time.sleep(interval)


# ---------------------------------------------------------------------------
# TCP server speaking the UDC framed chardev protocol
# ---------------------------------------------------------------------------
class MockChardevServer:
    """TCP server that behaves like the QEMU chardev socket endpoint.

    The same device instance is reused for sequential connections (its
    per-connection state is reset in on_open), which matches how tests use it:
    start once, then let clients connect / disconnect.
    """

    def __init__(
        self,
        device_factory: Callable[["MockChardevServer"], "MockDevice"],
        host: str = "127.0.0.1",
        port: int = 0,
        name: str = "mock-udc",
    ) -> None:
        self.device_factory = device_factory
        self.host = host
        self.requested_port = int(port)
        self.name = name
        self.port = 0
        self.device: Optional["MockDevice"] = None
        self.connections = 0
        self._listen: Optional[socket.socket] = None
        self._conn: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stopping = threading.Event()
        self._send_lock = threading.Lock()

    # -- lifecycle ----------------------------------------------------------
    def start(self) -> "MockChardevServer":
        self.device = self.device_factory(self)
        listen = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listen.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listen.bind((self.host, self.requested_port))
        listen.listen(4)
        listen.settimeout(0.1)
        self._listen = listen
        self.port = listen.getsockname()[1]
        self._thread = threading.Thread(target=self._serve, name=self.name, daemon=True)
        self._thread.start()
        LOG.info("mock server %s listening on %s:%d", self.name, self.host, self.port)
        return self

    def stop(self) -> None:
        self._stopping.set()
        conn = self._conn
        if conn is not None:
            try:
                conn.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                conn.close()
            except OSError:
                pass
        listen = self._listen
        if listen is not None:
            try:
                listen.close()
            except OSError:
                pass
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self._listen = None
        self._thread = None
        self._conn = None

    def __enter__(self) -> "MockChardevServer":
        return self.start()

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()

    # -- serving ------------------------------------------------------------
    def _serve(self) -> None:
        while not self._stopping.is_set():
            listen = self._listen
            if listen is None:
                return
            try:
                conn, _addr = listen.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            self.connections += 1
            self._serve_connection(conn)

    def _serve_connection(self, conn: socket.socket) -> None:
        self._conn = conn
        conn.settimeout(0.05)
        device = self.device
        assert device is not None
        decoder = vusb.FrameDecoder()
        try:
            self.send_frame(vusb.DEV_LOG, b"READY")
            device.on_open()
            while not self._stopping.is_set():
                try:
                    data = conn.recv(65536)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not data:
                    break
                for frame in decoder.feed(data):
                    device.on_frame(frame.type, frame.payload)
                if device.wants_close:
                    LOG.debug("device requested connection close")
                    break
        finally:
            try:
                device.on_close()
            except Exception:  # pragma: no cover - never mask test failures
                LOG.exception("mock device on_close failed")
            self._conn = None
            try:
                conn.close()
            except OSError:
                pass

    # -- frame I/O ----------------------------------------------------------
    def send_frame(self, frame_type: int, payload: bytes = b"") -> bool:
        conn = self._conn
        if conn is None:
            return False
        data = vusb.encode_frame(frame_type, payload)
        with self._send_lock:
            try:
                conn.sendall(data)
                return True
            except OSError:
                return False

    def send_log(self, text: str) -> bool:
        return self.send_frame(vusb.DEV_LOG, text.encode("latin-1"))


# ---------------------------------------------------------------------------
# base mock device
# ---------------------------------------------------------------------------
class MockDevice:
    """Base class: frame dispatch + helpers shared by the mock gadgets."""

    device_descriptor = build_device_descriptor()
    config_descriptor = build_config_descriptor()
    strings: Dict[int, str] = {}
    soft_conn_delay = 0.0
    qmu_ready_delay = 0.0
    require_soft_conn = True
    split_responses = False
    response_delay = 0.0
    reboot_closes = True
    #: Set to False to emulate a chardev opened *after* LK already pulled D+
    #: up (the model does not replay the SOFT_CONN log line).
    log_soft_conn = True

    def __init__(self, server: MockChardevServer) -> None:
        self.server = server
        self._lock = threading.RLock()
        self._cond = threading.Condition(self._lock)
        self.wants_close = False
        self.closed = False
        self.soft_conn = False
        self.connected = False
        self.configured = False
        self.address = 0
        self.setups: List[Tuple[int, int, int, int, int]] = []
        self.unknown_frames: List[Tuple[int, bytes]] = []
        self._reset_state()

    # -- hooks --------------------------------------------------------------
    def _reset_state(self) -> None:
        """Per-connection state (called from on_open)."""

    def on_open(self) -> None:
        with self._cond:
            self.wants_close = False
            self.closed = False
            self.soft_conn = False
            self.connected = False
            self.configured = False
            self.address = 0
            self.setups = []
            self.unknown_frames = []
            self._reset_state()
        if self.soft_conn_delay > 0:
            timer = threading.Timer(self.soft_conn_delay, self._raise_pullup)
            timer.daemon = True
            timer.start()
        else:
            self._raise_pullup()

    def _raise_pullup(self) -> None:
        with self._cond:
            self.soft_conn = True
            if self.log_soft_conn:
                self.send_log("SOFT_CONN")

    def on_close(self) -> None:
        with self._cond:
            self.closed = True
            self._cond.notify_all()

    def on_frame(self, frame_type: int, payload: bytes) -> None:
        if frame_type == vusb.HOST_CONNECT:
            with self._cond:
                self.connected = True
                self.send_log("CONNECTED")
        elif frame_type == vusb.HOST_SETUP:
            self.on_setup(payload)
        elif frame_type == vusb.HOST_EP1_OUT:
            self.on_bulk_out(payload)
        elif frame_type == vusb.HOST_EP0_ACK:
            pass
        elif frame_type == vusb.HOST_DISCONNECT:
            with self._cond:
                self.connected = False
        else:
            with self._cond:
                self.unknown_frames.append((frame_type, bytes(payload)))
            LOG.warning("mock device got unknown frame 0x%02x (%d bytes)",
                        frame_type, len(payload))

    def on_setup(self, payload: bytes) -> None:
        if len(payload) != 8:
            LOG.warning("mock got short SETUP payload: %s", payload.hex())
            return
        bm, request, value, index, length = struct.unpack("<BBHHH", payload)
        with self._cond:
            self.setups.append((bm, request, value, index, length))
        if self.require_soft_conn and not self.soft_conn:
            self.send_log("NOT_READY")
            return
        if (bm & vusb.USB_DIR_IN) and request == vusb.USB_REQ_GET_DESCRIPTOR:
            descriptor_type = value >> 8
            descriptor_index = value & 0xFF
            data = self._descriptor(descriptor_type, descriptor_index)
            if data is None:
                return
            if length:
                data = data[:length]
            self.send_ep0_in(data)
        elif request == vusb.USB_REQ_SET_ADDRESS:
            with self._cond:
                self.address = value & 0x7F
        elif request == vusb.USB_REQ_SET_CONFIGURATION:
            with self._cond:
                self.configured = True
            if self.qmu_ready_delay > 0:
                timer = threading.Timer(self.qmu_ready_delay, self.send_log, args=("QMU_READY",))
                timer.daemon = True
                timer.start()
            else:
                self.send_log("QMU_READY")

    def _descriptor(self, descriptor_type: int, index: int) -> Optional[bytes]:
        if descriptor_type == vusb.USB_DT_DEVICE:
            return self.device_descriptor
        if descriptor_type == vusb.USB_DT_CONFIG:
            return self.config_descriptor
        if descriptor_type == vusb.USB_DT_STRING:
            if index == 0:
                return bytes([4, vusb.USB_DT_STRING, 0x09, 0x04])  # lang 0x0409
            text = self.strings.get(index)
            if text is None:
                return None
            return build_string_descriptor(text)
        return None

    def on_bulk_out(self, payload: bytes) -> None:
        raise NotImplementedError

    # -- helpers ------------------------------------------------------------
    def send_log(self, text: str) -> None:
        self.server.send_log(text)

    def send_ep0_in(self, data: bytes) -> None:
        if data:
            self.server.send_frame(vusb.DEV_EP0_IN, data)

    def send_ep1_in(self, data: bytes) -> None:
        if self.response_delay > 0:
            time.sleep(self.response_delay)
        self.server.send_frame(vusb.DEV_EP1_IN, data)

    def send_fastboot_response(self, text: str) -> None:
        raw = text.encode("latin-1")
        if self.split_responses and len(raw) > 4:
            self.send_ep1_in(raw[:4])
            self.send_ep1_in(raw[4:])
        else:
            self.send_ep1_in(raw)

    def wait_for(self, predicate: Callable[[], bool], timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout))
        with self._cond:
            while True:
                if predicate():
                    return True
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._cond.wait(remaining)

    def _notify(self) -> None:
        with self._cond:
            self._cond.notify_all()


# ---------------------------------------------------------------------------
# fake LK fastboot device
# ---------------------------------------------------------------------------
class MockFastbootDevice(MockDevice):
    """Fake mtu3 UDC running LK fastboot (18d1:d00d)."""

    strings = {
        0: "0x0409",
        1: "Xiaomi",
        2: "xaga",
        3: "0123456789ABCDEF",
    }
    default_vars = {
        "version": "0.5",
        "product": "xaga",
        "current-slot": "a",
        "slot-count": "2",
        "max-download-size": "0x04000000",
        "serialno": "0xdeadbeef",
        "hw-revision": "mt6895",
        "secure": "yes",
        "unlocked": "no",
        "battery-voltage": "4200",
    }
    default_partitions = (
        "boot", "boot_a", "vendor_boot", "vendor_boot_a", "dtbo", "dtbo_a",
        "vbmeta", "vbmeta_a", "super", "userdata", "metadata", "logo",
    )

    #: When True send one DEV_EP1_ACK (0x83) frame per processed EP1 OUT frame,
    #: emulating the device-side RX-GPD completion ACK used for pacing.
    ack_frames = False
    #: "counter" (u32le ACK index), "bytes" (u32le cumulative bytes), "empty".
    ack_payload_mode = "counter"
    #: Stop sending ACKs after this many (None = never stop); for timeout tests.
    ack_drop_after = None
    #: Reply FAIL after a complete download payload (simulates LK USB read error).
    fail_download = False
    download_fail_message = "Read USB error"
    download_fail_info = ""

    def __init__(self, server: MockChardevServer) -> None:
        self.vars: Dict[str, str] = dict(self.default_vars)
        self.partition_names = set(self.default_partitions)
        self.fail_partitions: set = set()
        self.require_download_before_flash = True
        super().__init__(server)

    # -- per-connection state ----------------------------------------------
    def _reset_state(self) -> None:
        self.commands: List[bytes] = []
        self.downloads: List[bytes] = []
        self.flashes: List[Tuple[str, bytes]] = []
        self.last_download: Optional[bytes] = None
        self._download_size: Optional[int] = None
        self._download_buf = bytearray()
        self.acks_sent = 0
        self.ep1_out_bytes = 0

    # -- EP1 bulk OUT -------------------------------------------------------
    def on_bulk_out(self, payload: bytes) -> None:
        payload = bytes(payload)
        if self.require_soft_conn and not self.soft_conn:
            self.send_log("NOT_READY")
            return
        self.ep1_out_bytes += len(payload)
        # The model ACKs the RX GPD as soon as it is filled, i.e. before the
        # guest (LK) has parsed the payload and produced a fastboot reply.
        self._maybe_send_ack()
        with self._cond:
            downloading = self._download_size is not None
        if downloading:
            self._download_chunk(payload)
        else:
            command = payload.rstrip(b"\x00")
            if command:
                with self._cond:
                    self.commands.append(command)
                try:
                    text = command.decode("latin-1")
                except UnicodeDecodeError:  # pragma: no cover - commands are ASCII
                    self.send_fastboot_response("FAILbad command encoding")
                    text = None
                if text is not None:
                    self._handle_command(text)

    def _maybe_send_ack(self) -> None:
        """Emulate the model RX-GPD completion ACK (one 0x83 frame per EP1 OUT)."""
        if not self.ack_frames:
            return
        if self.ack_drop_after is not None and self.acks_sent >= self.ack_drop_after:
            return
        self.acks_sent += 1
        if self.ack_payload_mode == "empty":
            payload = b""
        elif self.ack_payload_mode == "bytes":
            payload = struct.pack("<I", self.ep1_out_bytes & 0xFFFFFFFF)
        else:
            payload = struct.pack("<I", self.acks_sent & 0xFFFFFFFF)
        self.server.send_frame(vusb.DEV_EP1_ACK, payload)

    def _download_chunk(self, payload: bytes) -> None:
        with self._cond:
            self._download_buf.extend(payload)
            complete = len(self._download_buf) >= (self._download_size or 0)
            image = bytes(self._download_buf[: self._download_size]) if complete else b""
            if complete:
                self.last_download = image
                self.downloads.append(image)
                self._download_size = None
                self._download_buf = bytearray()
        if complete:
            self._notify()
            if self.fail_download:
                if self.download_fail_info:
                    self.send_ep1_in(b"INFO" + self.download_fail_info.encode("latin-1"))
                self.send_fastboot_response("FAIL" + self.download_fail_message)
                return
            self.send_fastboot_response("OKAY")

    def _handle_command(self, text: str) -> None:
        if text.startswith("getvar:"):
            name = text[len("getvar:"):].strip()
            if name == "all":
                value = "".join("%s: %s\n" % (k, v) for k, v in self.vars.items())
                self.send_fastboot_response("OKAY" + value)
            elif name in self.vars:
                self.send_fastboot_response("OKAY" + self.vars[name])
            else:
                self.send_fastboot_response("FAILGetVar Variable not found")
        elif text.startswith("download:"):
            try:
                size = int(text[len("download:"):].strip(), 16)
            except ValueError:
                self.send_fastboot_response("FAILinvalid download size")
                return
            with self._cond:
                self._download_size = size
                self._download_buf = bytearray()
            self.send_fastboot_response("DATA%08x" % (size & 0xFFFFFFFF))
        elif text.startswith("flash:"):
            partition = text[len("flash:"):].strip()
            self._flash(partition)
        elif text.startswith("erase:"):
            self.send_fastboot_response("OKAY")
        elif text == "reboot" or text.startswith("reboot-"):
            self.send_fastboot_response("OKAY")
            if self.reboot_closes:
                with self._cond:
                    self.wants_close = True
        elif text.startswith("oem "):
            self.send_fastboot_response("OKAY")
        elif text.startswith("getvar"):
            self.send_fastboot_response("FAILunsupported command")
        else:
            self.send_fastboot_response("FAILunknown command")

    def _flash(self, partition: str) -> None:
        with self._cond:
            image = self.last_download
        if partition not in self.partition_names:
            self.send_fastboot_response("FAILpartition not found")
            return
        if partition in self.fail_partitions:
            self.send_fastboot_response("FAILwrite failure")
            return
        if image is None and self.require_download_before_flash:
            self.send_fastboot_response("FAILno image downloaded")
            return
        with self._cond:
            self.flashes.append((partition, image or b""))
            self.last_download = image
        self._notify()
        self.send_fastboot_response("OKAY")


# ---------------------------------------------------------------------------
# fake adb (USB ffs) device
# ---------------------------------------------------------------------------
class MockAdbDevice(MockDevice):
    """Fake adb gadget skeleton: CNXN handshake + basic OPEN/WRTE/OKAY."""

    device_descriptor = build_device_descriptor()
    config_descriptor = build_config_descriptor(
        interface_class=0xFF, interface_subclass=0x42, interface_protocol=0x01
    )
    strings = {0: "0x0409", 1: "Google", 2: "xaga", 3: "0123456789ABCDEF"}
    banner = (
        "device::ro.product.name=xaga;ro.product.model=Xaga;"
        "ro.product.device=xaga;features=cmd,shell_v2,stat_v2"
    )
    #: "none" answers CNXN to an AUTH packet (adb keys already accepted);
    #: "token" emulates a device that wants an RSA signature (TODO in adb.py).
    auth_mode = "none"

    def __init__(self, server: MockChardevServer) -> None:
        super().__init__(server)

    def _reset_state(self) -> None:
        self.packets: List[object] = []
        self.services: List[str] = []
        self.writes: List[bytes] = []
        self._reader = None
        self._next_local_id = 1
        self._next_remote_id = 101
        self._opens: Dict[int, int] = {}

    def on_bulk_out(self, payload: bytes) -> None:
        from . import adb  # local import keeps module import order simple

        if self._reader is None:
            self._reader = adb.AdbPacketReader()
        self._reader.feed(payload)
        while True:
            packet = self._reader.pop_packet()
            if packet is None:
                return
            with self._cond:
                self.packets.append(packet)
            self._handle_packet(packet)

    def _handle_packet(self, packet) -> None:
        from . import adb

        command = packet.command
        if command == adb.CMD_CNXN:
            if self.auth_mode != "none":
                # A device with key verification answers CNXN with AUTH(TOKEN).
                self.send_packet(
                    adb.AdbPacket(adb.CMD_AUTH, adb.AUTH_TOKEN, 0, bytes(range(20)))
                )
                return
            version = packet.arg0 or adb.A_VERSION
            maxdata = packet.arg1 or adb.A_MAXDATA
            self.send_packet(adb.AdbPacket(adb.CMD_CNXN, version, maxdata, self.banner.encode("ascii")))
        elif command == adb.CMD_AUTH:
            if self.auth_mode == "none":
                self.send_packet(
                    adb.AdbPacket(adb.CMD_CNXN, adb.A_VERSION, adb.A_MAXDATA, self.banner.encode("ascii"))
                )
            else:
                self.send_packet(
                    adb.AdbPacket(adb.CMD_AUTH, adb.AUTH_TOKEN, 0, bytes(range(20)))
                )
        elif command == adb.CMD_OPEN:
            service = packet.data.rstrip(b"\x00").decode("latin-1")
            with self._cond:
                local_id = self._next_local_id
                self._next_local_id += 1
                remote_id = self._next_remote_id
                self._next_remote_id += 1
                self._opens[local_id] = remote_id
                self.services.append(service)
            self.send_packet(adb.AdbPacket(adb.CMD_OKAY, local_id, remote_id))
        elif command == adb.CMD_WRTE:
            with self._cond:
                self.writes.append(bytes(packet.data))
            self.send_packet(adb.AdbPacket(adb.CMD_OKAY, packet.arg1, packet.arg0))
        elif command == adb.CMD_CLSE:
            self.send_packet(adb.AdbPacket(adb.CMD_CLSE, packet.arg1, packet.arg0))
        else:
            self.send_packet(adb.AdbPacket(adb.CMD_CLSE, packet.arg1, packet.arg0))

    def send_packet(self, packet) -> None:
        self.send_ep1_in(packet.encode())


# ---------------------------------------------------------------------------
# convenience factories
# ---------------------------------------------------------------------------
def fastboot_server(**kwargs) -> MockChardevServer:
    """Start a mock LK fastboot UDC server on an ephemeral localhost port."""
    return MockChardevServer(MockFastbootDevice, **kwargs).start()


def adb_server(**kwargs) -> MockChardevServer:
    """Start a mock adb UDC server on an ephemeral localhost port."""
    return MockChardevServer(MockAdbDevice, **kwargs).start()
