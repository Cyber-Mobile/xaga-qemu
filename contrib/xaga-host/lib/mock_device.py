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
    """Fake adb (USB ffs) gadget: CNXN/AUTH handshake + adb service multiplexer.

    Emulated services:
      * shell,v2,raw:<cmd> and shell:<cmd> (stdout/stderr/exit packet v2)
      * reboot:<target>
      * sideload-host:<size>:<block>  (minadbd block protocol)
      * sideload:<size>               (legacy push fallback)
      * sync:                         (push SEND/DATA/DONE)

    Failures can be injected with the attributes below, which makes the mock a
    regression target for the host CLI without QEMU.
    """

    device_descriptor = build_device_descriptor()
    config_descriptor = build_config_descriptor(
        interface_class=0xFF, interface_subclass=0x42, interface_protocol=0x01
    )
    strings = {0: "0x0409", 1: "Google", 2: "xaga", 3: "0123456789ABCDEF"}

    # -- configurable behaviour (persists across connections) ---------------
    banner_state = "device"
    banner_props = {
        "ro.product.name": "xaga",
        "ro.product.model": "Xaga",
        "ro.product.device": "xaga",
    }
    features = ("cmd", "shell_v2", "stat_v2")
    #: "none": accept CNXN immediately.  "token": send AUTH(TOKEN) and verify
    #: the RSA signature (or accept an AUTH(RSAPUBLICKEY) upload).
    auth_mode = "none"
    accept_public_key_upload = True
    #: Refuse sideload-host so the host must use the legacy push service.
    legacy_only = False
    #: Send FAILFAIL after the package was fully transferred.
    sideload_fail = False
    #: Optional Callable[[str], adb.ShellResult] used by the shell services.
    shell_handler = None

    def __init__(self, server: MockChardevServer) -> None:
        self.authorized_keys: List[Tuple[int, int]] = []
        self.banner_props = dict(self.__class__.banner_props)
        self.expected_sideload_sha256: Optional[str] = None
        self.expected_sideload_size: Optional[int] = None
        self.sync_files: Dict[str, bytes] = {}
        self.shell_handler = None
        super().__init__(server)

    # -- per-connection state ----------------------------------------------
    def _reset_state(self) -> None:
        self.packets: List[object] = []
        self.services: List[str] = []
        self.writes: List[bytes] = []
        self.shell_commands: List[str] = []
        self.auth_events: List[str] = []
        self.authenticated = False
        self.pending_token: Optional[bytes] = None
        self._reader = None
        self._next_remote_id = 101
        self.streams: Dict[int, Dict[str, object]] = {}
        self.reboot_target: Optional[str] = None
        self.sideload_protocol: Optional[str] = None
        self.sideload_bytes = 0
        self.sideload_blocks = 0
        self.sideload_sha256: Optional[str] = None
        self.sideload_ok: Optional[bool] = None
        self.sideload_error: Optional[str] = None
        self._sideload: Optional[Dict[str, object]] = None
        self._sync: Optional[Dict[str, object]] = None
        self.sync_received: Dict[str, bytes] = {}

    # -- helpers ------------------------------------------------------------
    def _banner_payload(self) -> bytes:
        props = ";".join(
            "%s=%s" % (key, value) for key, value in self.banner_props.items()
        )
        features = ",".join(self.features)
        return ("%s::%s;features=%s" % (self.banner_state, props, features)).encode(
            "latin-1"
        )

    def _send(self, packet) -> None:
        self.send_ep1_in(packet.encode())

    def _ack(self, stream: Dict[str, object]) -> None:
        from . import adb  # local import keeps module import order simple

        self._send(adb.AdbPacket(adb.A_OKAY, int(stream["remote_id"]),
                                 int(stream["host_id"])))

    def _close_stream(self, stream: Dict[str, object]) -> None:
        from . import adb

        self._send(adb.AdbPacket(adb.A_CLSE, int(stream["remote_id"]),
                                 int(stream["host_id"])))
        stream["state"] = "closed"

    def _send_stream_data(self, stream: Dict[str, object], data: bytes) -> None:
        from . import adb

        remote_id = int(stream["remote_id"])
        host_id = int(stream["host_id"])
        offset = 0
        chunk_size = 1024
        while offset < len(data):
            chunk = data[offset:offset + chunk_size]
            self._send(adb.AdbPacket(adb.A_WRTE, remote_id, host_id, chunk))
            offset += len(chunk)

    # -- packet dispatch ----------------------------------------------------
    def on_bulk_out(self, payload: bytes) -> None:
        from . import adb

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
        if command == adb.A_CNXN:
            self._on_cnxn(packet)
        elif command == adb.A_AUTH:
            self._on_auth(packet)
        elif command == adb.A_OPEN:
            self._on_open(packet)
        elif command == adb.A_WRTE:
            self._on_wrte(packet)
        elif command == adb.A_CLSE:
            self._on_clse(packet)
        elif command == adb.A_OKAY:
            return
        else:
            LOG.warning("mock adb got unknown packet 0x%08x", command)

    # -- handshake ----------------------------------------------------------
    def _accept(self) -> None:
        from . import adb

        maxdata = 4096
        self.authenticated = True
        self._send(adb.AdbPacket(adb.A_CNXN, adb.A_VERSION, maxdata,
                                 self._banner_payload()))

    def _new_token(self) -> bytes:
        self.pending_token = bytes(((index * 7 + 3) & 0xFF) for index in range(20))
        return self.pending_token

    def _send_token(self) -> None:
        from . import adb

        token = self._new_token()
        self.auth_events.append("auth_token")
        self._send(adb.AdbPacket(adb.A_AUTH, adb.AUTH_TOKEN, 0, token))

    def _on_cnxn(self, packet) -> None:
        if self.auth_mode == "none" or self.authenticated:
            self.auth_events.append("cnxn_accepted")
            self._accept()
        else:
            self._send_token()

    def _on_auth(self, packet) -> None:
        from . import adb

        if packet.arg0 == adb.AUTH_SIGNATURE:
            signature = bytes(packet.data)
            token = self.pending_token or b""
            accepted = any(
                adb.verify_sha1(n, e, token, signature)
                for n, e in self.authorized_keys
            )
            self.auth_events.append(
                "signature_ok" if accepted else "signature_rejected"
            )
            if accepted:
                self._accept()
            else:
                self._send_token()
            return
        if packet.arg0 == adb.AUTH_RSAPUBLICKEY:
            text = bytes(packet.data).rstrip(b"\x00").decode("latin-1")
            try:
                n, e = adb.decode_adb_public_key(text)
            except adb.AdbProtocolError as exc:
                self.auth_events.append("publickey_bad")
                LOG.warning("mock adb bad public key: %s", exc)
                self._send_token()
                return
            self.authorized_keys.append((n, e))
            self.auth_events.append("publickey")
            if self.accept_public_key_upload:
                self._accept()
            else:
                self._send_token()
            return
        self.auth_events.append("auth_kind_%d" % packet.arg0)

    # -- services -----------------------------------------------------------
    def _on_open(self, packet) -> None:
        from . import adb

        service = bytes(packet.data).rstrip(b"\x00").decode("latin-1")
        host_id = int(packet.arg0)
        remote_id = self._next_remote_id
        self._next_remote_id += 1
        stream: Dict[str, object] = {
            "host_id": host_id,
            "remote_id": remote_id,
            "service": service,
            "state": "open",
        }
        self.streams[host_id] = stream
        self.services.append(service)
        if service.startswith("sideload-host:") and self.legacy_only:
            # Old device: the service does not exist, adbd answers CLSE.
            self._close_stream(stream)
            return
        self._send(adb.AdbPacket(adb.A_OKAY, remote_id, host_id))
        self._start_service(stream)

    def _start_service(self, stream: Dict[str, object]) -> None:
        service = str(stream["service"])
        if service.startswith("shell,v2,raw:"):
            self._run_shell_v2(stream, service[len("shell,v2,raw:"):])
        elif service.startswith("shell:"):
            self._run_shell_legacy(stream, service[len("shell:"):])
        elif service.startswith("reboot:"):
            self.reboot_target = service[len("reboot:"):]
            self._close_stream(stream)
        elif service.startswith("sideload-host:"):
            self._start_sideload_host(stream, service[len("sideload-host:"):])
        elif service.startswith("sideload:"):
            size_text = service[len("sideload:"):]
            try:
                size = int(size_text)
            except ValueError:
                self._close_stream(stream)
                return
            self.sideload_protocol = "sideload"
            self._sideload = {
                "mode": "legacy",
                "stream": stream,
                "size": size,
                "received": 0,
                "blocks": 0,
                "hasher": __import__("hashlib").sha256(),
                "error": None,
            }
        elif service == "sync:":
            self._sync = {"path": None, "buffer": bytearray(), "rx": bytearray()}
        else:
            LOG.info("mock adb closing unsupported service %r", service)
            self._close_stream(stream)

    def _run_shell_legacy(self, stream: Dict[str, object], command: str) -> None:
        result = self._shell_result(command)
        self._send_stream_data(stream, result.output)
        self._close_stream(stream)

    def _run_shell_v2(self, stream: Dict[str, object], command: str) -> None:
        import struct as _struct

        result = self._shell_result(command)
        payload = bytearray()
        if result.stdout:
            payload += bytes([1]) + _struct.pack("<I", len(result.stdout)) + result.stdout
        if result.stderr:
            payload += bytes([2]) + _struct.pack("<I", len(result.stderr)) + result.stderr
        code = result.exit_code if result.exit_code is not None else 0
        payload += bytes([3]) + _struct.pack("<I", 1) + bytes([code & 0xFF])
        self._send_stream_data(stream, bytes(payload))
        self._close_stream(stream)

    def _shell_result(self, command: str):
        from . import adb

        self.shell_commands.append(command)
        if self.shell_handler is not None:
            return self.shell_handler(command)
        if command.startswith("echo "):
            return adb.ShellResult(command[5:].encode() + b"\n", b"", 0)
        if command.startswith("getprop "):
            prop = command[len("getprop "):].strip()
            values = {
                "ro.product.device": "xaga",
                "ro.serialno": "TESTSERIAL",
                "ro.build.version.release": "13",
            }
            value = values.get(prop, "")
            return adb.ShellResult((value + "\n").encode(), b"", 0)
        if command == "id":
            return adb.ShellResult(b"uid=0(root) gid=0(root)\n", b"", 0)
        return adb.ShellResult(b"", b"", 0)

    # -- sideload-host ------------------------------------------------------
    def _start_sideload_host(self, stream: Dict[str, object], args: str) -> None:
        import hashlib as _hashlib

        parts = args.split(":")
        if len(parts) != 2:
            self._close_stream(stream)
            return
        try:
            file_size = int(parts[0])
            block_size = int(parts[1])
        except ValueError:
            self._close_stream(stream)
            return
        if file_size <= 0 or block_size <= 0:
            self._close_stream(stream)
            return
        self.sideload_protocol = "sideload-host"
        self._sideload = {
            "mode": "host",
            "stream": stream,
            "size": file_size,
            "block": block_size,
            "next_block": 0,
            "received": 0,
            "blocks": 0,
            "hasher": _hashlib.sha256(),
            "error": None,
            "block_buf": bytearray(),
        }
        self._request_sideload_block(stream)

    def _request_sideload_block(self, stream: Dict[str, object]) -> None:
        state = self._sideload
        assert state is not None
        block = int(state["next_block"])
        self._send_stream_data(stream, b"%08d" % block)

    def _sideload_host_chunk(self, stream: Dict[str, object], data: bytes) -> None:
        state = self._sideload
        assert state is not None
        size = int(state["size"])
        block_size = int(state["block"])
        block = int(state["next_block"])
        offset = block * block_size
        expected = min(block_size, size - offset)
        buffer = state.get("block_buf")
        if buffer is None:
            buffer = bytearray()
            state["block_buf"] = buffer
        buffer.extend(data)
        if len(buffer) < expected:
            return
        if len(buffer) > expected:
            state["error"] = "block %d: got %d bytes, expected %d" % (
                block, len(buffer), expected)
            self._fail_sideload(stream)
            return
        payload = bytes(buffer)
        state["block_buf"] = bytearray()
        state["hasher"].update(payload)
        state["received"] = int(state["received"]) + len(payload)
        state["blocks"] = int(state["blocks"]) + 1
        state["next_block"] = block + 1
        total_blocks = (size + block_size - 1) // block_size
        if int(state["next_block"]) < total_blocks:
            self._request_sideload_block(stream)
            return
        self.sideload_bytes = int(state["received"])
        self.sideload_blocks = int(state["blocks"])
        self.sideload_sha256 = state["hasher"].hexdigest()
        ok = True
        if self.expected_sideload_size is not None and self.sideload_bytes != self.expected_sideload_size:
            ok = False
            self.sideload_error = "size mismatch: got %d expected %d" % (
                self.sideload_bytes, self.expected_sideload_size)
        if self.expected_sideload_sha256 is not None and self.sideload_sha256 != self.expected_sideload_sha256:
            ok = False
            self.sideload_error = "sha256 mismatch"
        if self.sideload_fail:
            ok = False
            self.sideload_error = "injected sideload failure"
        self.sideload_ok = ok
        if ok:
            self._send_stream_data(stream, b"DONEDONE")
        else:
            self._send_stream_data(stream, b"FAILFAIL")
        self._close_stream(stream)

    def _fail_sideload(self, stream: Dict[str, object]) -> None:
        state = self._sideload or {}
        self.sideload_bytes = int(state.get("received", 0))
        self.sideload_blocks = int(state.get("blocks", 0))
        hasher = state.get("hasher")
        if hasher is not None:
            self.sideload_sha256 = hasher.hexdigest()
        self.sideload_ok = False
        self._send_stream_data(stream, b"FAILFAIL")
        self._close_stream(stream)

    # -- legacy sideload ----------------------------------------------------
    def _finish_legacy_sideload(self) -> None:
        state = self._sideload or {}
        hasher = state.get("hasher")
        self.sideload_protocol = "sideload"
        self.sideload_bytes = int(state.get("received", 0))
        self.sideload_blocks = int(state.get("blocks", 0))
        if hasher is not None:
            self.sideload_sha256 = hasher.hexdigest()
        ok = True
        if self.expected_sideload_size is not None and self.sideload_bytes != self.expected_sideload_size:
            ok = False
            self.sideload_error = "size mismatch"
        if self.expected_sideload_sha256 is not None and self.sideload_sha256 != self.expected_sideload_sha256:
            ok = False
            self.sideload_error = "sha256 mismatch"
        if self.sideload_fail:
            ok = False
            self.sideload_error = "injected sideload failure"
        self.sideload_ok = ok

    # -- data / close -------------------------------------------------------
    def _on_wrte(self, packet) -> None:
        from . import adb

        # host -> device WRTE carries (host local id, device remote id)
        host_id = int(packet.arg0)
        stream = self.streams.get(host_id)
        data = bytes(packet.data)
        with self._cond:
            self.writes.append(data)
        if stream is None:
            LOG.warning("mock adb WRTE for unknown stream %d", host_id)
            self._send(adb.AdbPacket(adb.A_CLSE, 0, host_id))
            return
        service = str(stream["service"])
        self._ack(stream)
        if service.startswith("sideload-host:") and self._sideload is not None:
            self._sideload_host_chunk(stream, data)
        elif service.startswith("sideload:") and self._sideload is not None:
            state = self._sideload
            state["hasher"].update(data)
            state["received"] = int(state["received"]) + len(data)
            state["blocks"] = int(state["blocks"]) + 1
        elif service == "sync:":
            self._sync_request(stream, data)

    def _on_clse(self, packet) -> None:
        from . import adb

        # host -> device CLSE carries (host local id, device remote id)
        host_id = int(packet.arg0)
        stream = self.streams.get(host_id)
        if stream is None:
            return
        service = str(stream["service"])
        if service.startswith("sideload:") and self._sideload is not None \
                and self.sideload_protocol == "sideload":
            self._finish_legacy_sideload()
        self._send(adb.AdbPacket(adb.A_CLSE, int(stream["remote_id"]), host_id))
        stream["state"] = "closed"

    # -- sync push ----------------------------------------------------------
    def _sync_request(self, stream: Dict[str, object], data: bytes) -> None:
        state = self._sync or {"path": None, "buffer": bytearray(), "rx": bytearray()}
        self._sync = state
        rx = state["rx"]
        rx.extend(data)
        while len(rx) >= 4:
            command = bytes(rx[:4])
            if command == b"DONE":
                if len(rx) < 8:
                    return
                del rx[:8]  # DONE carries a 4-byte mtime, not a length
                path = state.get("path") or ""
                self.sync_received[str(path)] = bytes(state["buffer"])
                self._send_stream_data(stream, b"OKAY" + struct.pack("<I", 0))
                continue
            if len(rx) < 8:
                return
            length = struct.unpack("<I", bytes(rx[4:8]))[0]
            if len(rx) < 8 + length:
                return
            payload = bytes(rx[8:8 + length])
            del rx[:8 + length]
            if command == b"SEND":
                state["path"] = payload.decode("utf-8", "replace")
                state["buffer"] = bytearray()
                self._send_stream_data(stream, b"OKAY" + struct.pack("<I", 0))
            elif command == b"DATA":
                state["buffer"].extend(payload)
            else:
                self._send_stream_data(stream, b"FAIL" + struct.pack("<I", 0))

    # -- test helpers -------------------------------------------------------
    def expect_sideload(self, data: bytes) -> None:
        """Set the package the next sideload must match byte for byte."""
        import hashlib as _hashlib

        self.expected_sideload_size = len(data)
        self.expected_sideload_sha256 = _hashlib.sha256(data).hexdigest()



# ---------------------------------------------------------------------------
# convenience factories
# ---------------------------------------------------------------------------
def fastboot_server(**kwargs) -> MockChardevServer:
    """Start a mock LK fastboot UDC server on an ephemeral localhost port."""
    return MockChardevServer(MockFastbootDevice, **kwargs).start()


def adb_server(**kwargs) -> MockChardevServer:
    """Start a mock adb UDC server on an ephemeral localhost port."""
    return MockChardevServer(MockAdbDevice, **kwargs).start()
