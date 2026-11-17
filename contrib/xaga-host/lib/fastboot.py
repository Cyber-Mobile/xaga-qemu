"""Fastboot client for the MT6895 QEMU virtual UDC bridge.

This is the high level API used by host/fastboot_cli.py.  It drives the whole
LK bring-up sequence and then speaks the standard fastboot protocol over the
emulated EP1 bulk endpoints:

    connect chardev
      -> wait LOG READY
      -> wait LOG SOFT_CONN              (LK raised the U2 pull-up)
      -> HOST_CONNECT + settle           (bus reset seen by LK)
      -> GET_DESCRIPTOR / SET_ADDRESS / GET_DESCRIPTOR / SET_CONFIGURATION
      -> wait LOG QMU_READY              (LK armed the EP1 QMU queues)
      -> getvar:<name> / download:<size> / flash:<partition> / reboot / oem

Responses use the standard fastboot framing: a 4-byte status token
(OKAY / FAIL / DATA / INFO) followed by optional text.  The virtual UDC model
delivers them as one or more 0x82 EP1 IN frames, so replies are reassembled.

Only the Python standard library is used.
"""
from __future__ import annotations

import dataclasses
import logging
import os
import time
from collections import deque
from typing import Callable, Deque, List, Optional

from . import vusb

LOG = logging.getLogger("vusb.fastboot")


class FastbootError(vusb.VusbError):
    """Base fastboot client error."""


@dataclasses.dataclass
class DownloadDiagnostics:
    """Snapshot of a download transfer, attached to failures for reporting."""

    total_bytes: int = 0
    chunk_size: int = 0
    pacing: str = "delay"
    window: int = 0
    sent_bytes: int = 0
    chunks_sent: int = 0
    acks_received: int = 0
    last_ack: str = "-"
    info_lines: List[str] = dataclasses.field(default_factory=list)
    last_events: List[str] = dataclasses.field(default_factory=list)
    elapsed: float = 0.0
    stage: str = "init"

    @property
    def percent(self) -> float:
        if self.total_bytes <= 0:
            return 100.0
        return 100.0 * self.sent_bytes / self.total_bytes

    def describe(self) -> List[str]:
        lines = [
            "download stage=%s pacing=%s%s chunk=%d"
            % (self.stage, self.pacing,
               (" window=%d" % self.window) if self.pacing == "ack" else "",
               self.chunk_size),
            "sent %d/%d bytes (%d chunks, %.1f%%) in %.2fs"
            % (self.sent_bytes, self.total_bytes, self.chunks_sent,
               self.percent, self.elapsed),
            "acks=%d last_ack=%s" % (self.acks_received, self.last_ack),
        ]
        if self.info_lines:
            lines.append("info lines=%d: %s"
                         % (len(self.info_lines),
                            " | ".join(line.replace("\n", " ")[:160]
                                       for line in self.info_lines)))
        else:
            lines.append("info lines=0")
        if self.last_events:
            lines.append("last events:")
            lines.extend("  " + event for event in self.last_events[-8:])
        return lines


class FastbootFail(FastbootError):
    """The device answered with a FAIL status (text is in .message)."""

    def __init__(self, message: str,
                 details: Optional[DownloadDiagnostics] = None) -> None:
        self.message = message
        self.details = details
        super().__init__("fastboot FAIL: %s" % message)


class FastbootDownloadError(FastbootError):
    """Transport/ACK/timeout failure while sending a download payload."""

    def __init__(self, message: str,
                 details: Optional[DownloadDiagnostics] = None) -> None:
        self.details = details
        super().__init__(message)


class FastbootProtocolError(FastbootError):
    """The device answered with something that is not valid fastboot."""


@dataclasses.dataclass
class FastbootResponse:
    """One parsed fastboot reply."""

    status: str
    data: bytes = b""

    @property
    def text(self) -> str:
        return self.data.decode("latin-1")

    def __str__(self) -> str:
        return "%s%s" % (self.status, self.text)


ProgressCallback = Callable[[int, int, float], None]


class FastbootClient:
    """Fastboot protocol client for the QEMU virtual UDC chardev.

    Parameters are tuning knobs for the real model; tests use much smaller
    settle values.  Defaults mirror the delays that made the LK UDC model
    reliable in the r1295 bring-up runs.
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 4444,
        *,
        connect_timeout: float = 30.0,
        io_timeout: float = 10.0,
        soft_conn_timeout: float = 150.0,
        pullup_settle: float = 0.5,
        bus_settle: float = 1.0,
        descriptor_settle: float = 0.25,
        qmu_timeout: float = 5.0,
        command_timeout: float = 10.0,
        download_timeout: float = 120.0,
        flash_timeout: float = 180.0,
        chunk_size: int = vusb.EP1_CHUNK_SIZE,
        chunk_delay: float = 0.0,
        response_idle: float = 0.05,
        enum_attempts: int = 4,
        configure_attempts: int = 4,
        pacing: str = "delay",
        ack_window: int = 4,
        ack_timeout: float = 10.0,
        ack_frame_type: int = vusb.DEV_EP1_ACK,
        record_traffic: bool = False,
    ) -> None:
        self.host = host
        self.port = int(port)
        pacing_mode = str(pacing).lower()
        if pacing_mode not in ("delay", "ack"):
            raise ValueError("pacing must be 'delay' or 'ack', got %r" % (pacing,))
        self.pacing = pacing_mode
        self.ack_window = max(1, int(ack_window))
        self.ack_timeout = float(ack_timeout)
        self.usb = vusb.VUsbHost(
            host, port, connect_timeout=connect_timeout, io_timeout=io_timeout,
            ack_frame_type=ack_frame_type, record_traffic=record_traffic,
        )
        self.soft_conn_timeout = soft_conn_timeout
        self.pullup_settle = pullup_settle
        self.bus_settle = bus_settle
        self.descriptor_settle = descriptor_settle
        self.qmu_timeout = qmu_timeout
        self.command_timeout = command_timeout
        self.download_timeout = download_timeout
        self.flash_timeout = flash_timeout
        self.chunk_size = int(chunk_size)
        self.chunk_delay = float(chunk_delay)
        self.response_idle = float(response_idle)
        self.enum_attempts = max(1, int(enum_attempts))
        self.configure_attempts = max(1, int(configure_attempts))
        self.device: Optional[vusb.UsbDeviceInfo] = None
        self.connected = False
        self._t0_client = time.monotonic()
        self._events: Deque[str] = deque(maxlen=64)
        self.info_lines: List[str] = []
        self.last_download_diagnostics: Optional[DownloadDiagnostics] = None
        self._max_download_size_known = False
        self._max_download_size: Optional[int] = None

    # -- lifecycle ----------------------------------------------------------
    def connect(self) -> vusb.UsbDeviceInfo:
        """Open the chardev and bring the LK fastboot gadget up."""
        self.usb.connect()
        try:
            if not self.usb.wait_ready(5.0):
                LOG.warning("no LOG READY from chardev (continuing anyway)")
            try:
                self.usb.wait_pullup(
                    timeout=self.soft_conn_timeout, settle=self.pullup_settle
                )
            except vusb.VusbTimeout as exc:
                # The chardev does not replay SOFT_CONN if the host connects
                # after LK already raised D+ up.  Probe with enumeration
                # instead of failing straight away.
                LOG.warning("%s; probing enumeration anyway", exc)
            last_error: Optional[BaseException] = None
            for attempt in range(1, self.enum_attempts + 1):
                self.usb.connect_bus(wait_connected=1.0)
                if self.bus_settle > 0:
                    self.usb.pump(self.bus_settle)
                try:
                    device = self.usb.enumerate(
                        descriptor_settle=self.descriptor_settle, configure=True
                    )
                except vusb.VusbError as exc:
                    last_error = exc
                    LOG.warning("enumeration attempt %d/%d failed: %s",
                                attempt, self.enum_attempts, exc)
                    continue
                if self._wait_qmu_ready():
                    self.device = device
                    self.connected = True
                    LOG.info("fastboot gadget ready: %s", device.summary())
                    return device
                LOG.info("QMU_READY not seen, re-sending SET_CONFIGURATION")
                for cfg_attempt in range(self.configure_attempts):
                    self.usb.set_configuration(self._config_value(device))
                    if self._wait_qmu_ready():
                        self.device = device
                        self.connected = True
                        LOG.info("fastboot gadget ready after SET_CONFIGURATION retry %d",
                                 cfg_attempt + 1)
                        return device
                last_error = vusb.VusbTimeout(
                    "no LOG QMU_READY after SET_CONFIGURATION"
                )
            raise FastbootError("fastboot bring-up failed: %s" % (last_error,))
        except BaseException:
            self.usb.close()
            raise

    @staticmethod
    def _config_value(device: vusb.UsbDeviceInfo) -> int:
        if device.config is not None and device.config.bConfigurationValue:
            return device.config.bConfigurationValue
        return 1

    def _wait_qmu_ready(self) -> bool:
        return self.usb.wait_log("QMU_READY", self.qmu_timeout)

    def close(self) -> None:
        self.usb.close()
        self.connected = False

    def __enter__(self) -> "FastbootClient":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # -- low level protocol -------------------------------------------------
    def _read_response(self, timeout: Optional[float] = None) -> FastbootResponse:
        limit = self.command_timeout if timeout is None else float(timeout)
        deadline = time.monotonic() + limit
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise vusb.VusbTimeout("timed out waiting for a fastboot reply")
            frames = self.usb.bulk_in_frames(
                timeout=remaining, idle_gap=self.response_idle
            )
            aggregated = b""
            for chunk in frames:
                if not aggregated and chunk[:4] == b"INFO":
                    text = chunk[4:].decode("latin-1")
                    self.info_lines.append(text)
                    self._event("INFO %s" % text.replace("\n", " ")[:120])
                    LOG.info("device INFO: %s", text)
                    continue
                aggregated += chunk
            if not aggregated:
                continue
            response = _parse_response(aggregated)
            self._event("resp %s len=%d" % (response.status, len(response.data)))
            return response

    def command(self, text: str, *, timeout: Optional[float] = None) -> FastbootResponse:
        """Send one raw fastboot command and return the parsed reply."""
        payload = text.encode("latin-1")
        if not payload:
            raise ValueError("empty fastboot command")
        LOG.debug("-> %s", text)
        self._event("cmd %s" % text[:120])
        self.usb.bulk_out_raw(payload)
        response = self._read_response(timeout)
        LOG.debug("<- %s", response)
        return response

    def getvar(self, name: str, *, timeout: Optional[float] = None) -> str:
        """Return a fastboot variable value (without the OKAY prefix)."""
        response = self.command("getvar:%s" % name, timeout=timeout)
        if response.status != "OKAY":
            raise FastbootProtocolError("unexpected getvar reply: %s" % response)
        return response.text

    def _event(self, text: str) -> None:
        elapsed = time.monotonic() - self._t0_client
        self._events.append("%8.2fs %s" % (elapsed, text))

    def get_max_download_size(self, *, refresh: bool = False) -> Optional[int]:
        """Read + parse "getvar:max-download-size" (hex).  None if unknown."""
        if self._max_download_size_known and not refresh:
            return self._max_download_size
        self._max_download_size_known = True
        try:
            text = self.getvar("max-download-size")
        except FastbootFail as exc:
            LOG.info("device does not report max-download-size: %s", exc.message)
            self._max_download_size = None
            return None
        text = text.strip()
        if not text:
            self._max_download_size = None
            return None
        try:
            value = int(text, 16)
        except ValueError:
            LOG.warning("cannot parse max-download-size %r as hex", text)
            self._max_download_size = None
            return None
        self._max_download_size = value if value > 0 else None
        if self._max_download_size is None:
            LOG.info("device max-download-size: unknown (%r)", text)
        else:
            LOG.info("device max-download-size: %s (%d bytes, 0x%x)",
                     _human_size(self._max_download_size),
                     self._max_download_size, self._max_download_size)
        return self._max_download_size

    def ensure_downloadable(self, byte_count: int) -> Optional[int]:
        """Fail early when the image exceeds the device max-download-size."""
        limit = self.get_max_download_size()
        if limit is not None and byte_count > limit:
            raise FastbootError(
                "image too large: %s (%d bytes) exceeds device max-download-size "
                "%s (%d bytes / 0x%x)"
                % (_human_size(byte_count), byte_count,
                   _human_size(limit), limit, limit)
            )
        if limit is not None:
            self._event("max-download-size %d bytes, image %d bytes fits"
                        % (limit, byte_count))
        return limit

    # -- data transfer ------------------------------------------------------
    def download(
        self,
        data: bytes,
        *,
        progress: Optional[ProgressCallback] = None,
        chunk_size: Optional[int] = None,
        chunk_delay: Optional[float] = None,
        timeout: Optional[float] = None,
        pacing: Optional[str] = None,
        ack_window: Optional[int] = None,
        ack_timeout: Optional[float] = None,
    ) -> vusb.TransferStats:
        """fastboot download:<size> + DATA handshake + bulk data + OKAY.

        Pacing is either "delay" (fixed sleep between chunks, works on the
        first-generation model) or "ack" (credit flow control driven by the
        model's 0x83 RX-GPD ACK frames).  Failures carry a
        :class:'DownloadDiagnostics' snapshot (sent bytes, ACKs, INFO lines)
        for post-mortem printing.
        """
        data = bytes(data)
        size = len(data)
        chunk = self.chunk_size if chunk_size is None else int(chunk_size)
        pacing_mode = self.pacing if pacing is None else str(pacing).lower()
        if pacing_mode not in ("delay", "ack"):
            raise ValueError("pacing must be 'delay' or 'ack', got %r" % (pacing,))
        window = self.ack_window if ack_window is None else max(1, int(ack_window))
        diag = DownloadDiagnostics(
            total_bytes=size,
            chunk_size=chunk,
            pacing=pacing_mode,
            window=window if pacing_mode == "ack" else 0,
        )
        self.last_download_diagnostics = diag
        self.info_lines = []
        started = time.monotonic()
        ack_base = 0
        self._event("download:%08x begin pacing=%s%s"
                    % (size, pacing_mode,
                       (" window=%d" % window) if pacing_mode == "ack" else ""))

        def _progress(done: int, total: int, elapsed: float) -> None:
            diag.sent_bytes = done
            diag.chunks_sent = (done + chunk - 1) // chunk if chunk > 0 else 0
            diag.elapsed = time.monotonic() - started
            if progress is not None:
                progress(done, total, elapsed)

        try:
            diag.stage = "data-handshake"
            response = self.command("download:%08x" % size)
            if response.status != "DATA":
                raise FastbootProtocolError(
                    "expected DATA reply for download:%08x, got %s" % (size, response)
                )
            declared_text = response.text.strip()
            try:
                declared = int(declared_text, 16)
            except ValueError:
                raise FastbootProtocolError(
                    "device sent malformed DATA reply: %r" % declared_text
                )
            if declared != size:
                raise FastbootProtocolError(
                    "device asked for %d bytes but host has %d" % (declared, size)
                )
            diag.stage = "payload"
            # ACKs are only counted from the start of the payload: the ACK for
            # the download command frame belongs to the DATA handshake.
            ack_base = self.usb.ack_count
            if pacing_mode == "ack":
                stats = self.usb.bulk_out_paced(
                    data,
                    window=window,
                    ack_timeout=(self.ack_timeout
                                 if ack_timeout is None else float(ack_timeout)),
                    chunk_size=chunk,
                    progress=_progress,
                )
            else:
                stats = self.usb.bulk_out(
                    data,
                    chunk_size=chunk,
                    chunk_delay=(self.chunk_delay
                                 if chunk_delay is None else float(chunk_delay)),
                    progress=_progress,
                )
            diag.sent_bytes = stats.byte_count
            diag.chunks_sent = stats.chunk_count
            diag.acks_received = stats.acks
            diag.last_ack = str(self.usb.last_ack) if self.usb.last_ack else "-"
            diag.elapsed = time.monotonic() - started
            diag.stage = "await-okay"
            self._event("payload sent %d bytes in %.2fs; waiting for OKAY"
                        % (size, diag.elapsed))
            response = self._read_response(
                self.download_timeout if timeout is None else timeout
            )
            if response.status != "OKAY":
                raise FastbootProtocolError(
                    "unexpected reply after download payload: %s" % response
                )
            diag.stage = "done"
            return stats
        except FastbootFail as exc:
            diag.stage = "device-FAIL"
            self._fill_diag(diag, started, ack_base)
            self._event("FAIL %s" % exc.message)
            if exc.details is None:
                exc.details = diag
            raise
        except vusb.VusbTimeout as exc:
            diag.stage = "timeout"
            self._fill_diag(diag, started, ack_base)
            raise FastbootDownloadError(
                "download payload failed: %s" % exc, details=diag
            ) from exc
        except FastbootError as exc:
            diag.stage = "protocol-error"
            self._fill_diag(diag, started, ack_base)
            if getattr(exc, "details", None) is None:
                exc.details = diag  # type: ignore[attr-defined]
            raise

    def _fill_diag(self, diag: DownloadDiagnostics, started: float,
                   ack_base: int) -> None:
        diag.acks_received = self.usb.ack_count - ack_base
        diag.last_ack = str(self.usb.last_ack) if self.usb.last_ack else "-"
        diag.info_lines = list(self.info_lines)
        diag.last_events = list(self._events)
        diag.elapsed = time.monotonic() - started

    def flash(
        self,
        partition: str,
        data: Optional[bytes] = None,
        *,
        path: Optional[str] = None,
        progress: Optional[ProgressCallback] = None,
        chunk_size: Optional[int] = None,
        chunk_delay: Optional[float] = None,
        download_timeout: Optional[float] = None,
        flash_timeout: Optional[float] = None,
    ) -> FastbootResponse:
        """Download an image (bytes or file) and flash it to *partition*."""
        if (data is None) == (path is None):
            raise ValueError("flash() needs exactly one of data= or path=")
        if path is not None:
            data = read_image_file(path)
        assert data is not None
        self.ensure_downloadable(len(data))
        self.download(
            data,
            progress=progress,
            chunk_size=chunk_size,
            chunk_delay=chunk_delay,
            timeout=download_timeout,
        )
        response = self.command(
            "flash:%s" % partition, timeout=flash_timeout or self.flash_timeout
        )
        if response.status != "OKAY":
            raise FastbootProtocolError(
                "unexpected flash reply for %s: %s" % (partition, response)
            )
        return response

    def erase(self, partition: str, *, timeout: Optional[float] = None) -> FastbootResponse:
        return self.command("erase:%s" % partition, timeout=timeout)

    # -- misc commands ------------------------------------------------------
    def reboot(self, mode: str = "", *, timeout: Optional[float] = None) -> Optional[FastbootResponse]:
        """reboot / reboot-bootloader / reboot-fastboot / reboot-recovery."""
        commands = {
            "": "reboot",
            "normal": "reboot",
            "bootloader": "reboot-bootloader",
            "fastboot": "reboot-fastboot",
            "recovery": "reboot-recovery",
        }
        key = mode.lower()
        if key not in commands:
            raise ValueError(
                "unknown reboot mode %r (use bootloader/fastboot/recovery)" % (mode,)
            )
        command = commands[key]
        try:
            return self.command(command, timeout=timeout or self.command_timeout)
        except vusb.VusbConnectionError as exc:
            # A device that resets immediately may close the chardev before
            # the reply arrives; that is not necessarily an error.
            LOG.warning("connection closed during %s: %s", command, exc)
            return None

    def oem(self, *args: str, timeout: Optional[float] = None) -> FastbootResponse:
        command = "oem " + " ".join(str(arg) for arg in args)
        return self.command(command, timeout=timeout)


def _human_size(count: int) -> str:
    value = float(count)
    if value < 1024.0:
        return "%d B" % int(value)
    for unit in ("KiB", "MiB", "GiB"):
        value /= 1024.0
        if value < 1024.0 or unit == "GiB":
            return "%.1f %s" % (value, unit)
    return "%d B" % count  # pragma: no cover - unreachable


def parse_response(raw: bytes) -> FastbootResponse:
    """Parse one fastboot reply payload into a FastbootResponse."""
    return _parse_response(raw)


def _parse_response(raw: bytes) -> FastbootResponse:
    raw = bytes(raw)
    if len(raw) < 4:
        raise FastbootProtocolError(
            "fastboot reply shorter than 4 bytes: %s" % raw.hex()
        )
    status = raw[:4].decode("latin-1")
    body = raw[4:]
    if status == "OKAY":
        return FastbootResponse("OKAY", body)
    if status == "FAIL":
        raise FastbootFail(body.decode("latin-1"))
    if status == "DATA":
        return FastbootResponse("DATA", body)
    if status == "INFO":
        return FastbootResponse("INFO", body)
    raise FastbootProtocolError(
        "unknown fastboot status %r in reply %s" % (status, raw.hex())
    )


def read_image_file(path: str) -> bytes:
    """Read a local image file (boot.img, vendor_boot.img, dtbo.img, ...)."""
    path = os.path.expanduser(path)
    try:
        with open(path, "rb") as handle:
            return handle.read()
    except OSError as exc:
        raise FastbootError("cannot read image file %s: %s" % (path, exc)) from exc
