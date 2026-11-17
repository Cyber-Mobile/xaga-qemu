"""adb client for the mt6895 virtual USB gadget (USB ffs transport).

This module implements the host side of the AOSP adb wire protocol on top of
the virtual UDC chardev transport (see lib/vusb.py).  It is dependency-free
(Python standard library only; openssl is used only as an optional fallback
for RSA key generation / signing when a key cannot be parsed in pure Python).

Implemented
-----------
* packet framing: 24-byte header, magic, byte-sum checksum
* CNXN handshake with banner/property/feature parsing
* RSA AUTH: SHA-1 + PKCS#1 v1.5 signature for AUTH(TOKEN); when the device
  does not know the host key yet, upload AUTH(RSAPUBLICKEY) in Android's
  524-byte little-endian key format
* stream multiplexing: OPEN / WRTE / OKAY / CLSE with per-stream flow control
* shell: shell protocol v2 ("shell,v2,raw:<cmd>") with fallback to the legacy
  "shell:<cmd>" service
* reboot: "reboot:<target>" service (default/bootloader/recovery/fastboot/rescue)
* sideload: the current AOSP recovery protocol "sideload-host:<size>:<block>"
  (device requests 8-digit block indices, host serves 64 KiB blocks, device
  answers DONEDONE/FAILFAIL) plus the legacy "sideload:<size>" push fallback
* optional "sync:" push (SEND/DATA/DONE) for adb push

Not implemented (documented as TODO)
------------------------------------
* pull (sync RECV), stat/list, incremental install, rescue-* services
* shell protocol v2 stdin streaming and window-size packets
* adb TLS (STLS) / pairing

Protocol references: AOSP system/core adb protocol.txt, packages/modules/adb
client/commandline.cpp (adb_sideload_install), crypto utilities
android_pubkey.cpp, and LineageOS recovery minadbd (minadbd_services.cpp,
fuse_adb_provider.cpp).
"""
from __future__ import annotations

import base64
import dataclasses
import hashlib
import logging
import os
import shutil
import struct
import subprocess
import tempfile
import time
from typing import Callable, Dict, List, Optional, Tuple

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

#: AOSP adb source spells the command constants CMD_*; keep both spellings.
CMD_SYNC = A_SYNC
CMD_CNXN = A_CNXN
CMD_OPEN = A_OPEN
CMD_OKAY = A_OKAY
CMD_CLSE = A_CLSE
CMD_WRTE = A_WRTE
CMD_AUTH = A_AUTH
CMD_STLS = A_STLS

AUTH_TOKEN = 1
AUTH_SIGNATURE = 2
AUTH_RSAPUBLICKEY = 3

HEADER = struct.Struct("<6I")
MAX_PACKET_DATA = 1024 * 1024
HOST_BANNER = "host::features=cmd,shell_v2,stat_v2"

#: minadbd sends these 8-byte strings instead of a block number when done.
SIDELOAD_EXIT_SUCCESS = b"DONEDONE"
SIDELOAD_EXIT_FAILURE = b"FAILFAIL"

#: Default block size advertised in the sideload-host service name.  AOSP uses
#: CHUNK_SIZE (64 KiB); minadbd uses it as the FUSE read block size and derives
#: every requested block index from it.
SIDELOAD_BLOCK_SIZE = 65536

#: Android custom RSA public key binary format (crypto_utils/android_pubkey.h).
ANDROID_PUBKEY_MODULUS_SIZE = 256
ANDROID_PUBKEY_ENCODED_SIZE = 3 * 4 + 2 * ANDROID_PUBKEY_MODULUS_SIZE
ANDROID_PUBKEY_MODULUS_WORDS = ANDROID_PUBKEY_MODULUS_SIZE // 4

#: DER prefix of the PKCS#1 v1.5 DigestInfo for SHA-1 (RFC 3447 / RFC 8017).
_SHA1_DIGEST_INFO_PREFIX = bytes.fromhex("3021300906052b0e03021a05000414")

DEFAULT_KEY_PATH = os.path.join("~", ".android", "adbkey")


class AdbError(vusb.VusbError):
    """Base adb client error."""


class AdbProtocolError(AdbError):
    """Malformed or unexpected adb packet."""


class AdbAuthError(AdbError):
    """The device refused the host authentication / no usable key."""


class AdbStreamError(AdbError):
    """A stream failed (refused, reset or timed out)."""


class AdbStreamClosed(AdbStreamError):
    """The remote side closed the stream."""


class AdbKeyError(AdbError):
    """The adb private key could not be loaded/generated/signed."""


class SideloadError(AdbError):
    """sideload transfer failed."""


# ---------------------------------------------------------------------------
# packets
# ---------------------------------------------------------------------------
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


# ---------------------------------------------------------------------------
# banner
# ---------------------------------------------------------------------------
@dataclasses.dataclass
class AdbBanner:
    """Parsed CNXN banner, e.g. device::ro.product.device=xaga;features=... ."""

    state: str = "device"
    props: Dict[str, str] = dataclasses.field(default_factory=dict)
    features: set = dataclasses.field(default_factory=set)
    maxdata: int = A_MAXDATA
    version: int = A_VERSION
    raw: str = ""

    @classmethod
    def parse(cls, payload: bytes, arg0: int = A_VERSION, arg1: int = A_MAXDATA) -> "AdbBanner":
        text = payload.decode("latin-1").rstrip("\x00")
        banner = cls(version=arg0 or A_VERSION, maxdata=arg1 or A_MAXDATA, raw=text)
        state, sep, rest = text.partition("::")
        if not sep:
            state, rest = "device", text
        banner.state = state or "device"
        for item in rest.split(";"):
            item = item.strip()
            if not item:
                continue
            key, _, value = item.partition("=")
            if not key:
                continue
            if key == "features":
                banner.features = set(part for part in value.split(",") if part)
            else:
                banner.props[key] = value
        return banner

    @property
    def serial(self) -> str:
        return (
            self.props.get("ro.serialno")
            or self.props.get("ro.boot.serialno")
            or self.props.get("ro.product.device")
            or "unknown"
        )

    @property
    def device_name(self) -> str:
        return self.props.get("ro.product.device") or self.props.get("ro.product.name") or "unknown"


# ---------------------------------------------------------------------------
# RSA / adb key support (pure Python, openssl only as fallback)
# ---------------------------------------------------------------------------
def _der_read(data: bytes, offset: int) -> Tuple[int, bytes, int]:
    """Read one DER TLV; returns (tag, value, next_offset)."""
    if offset + 2 > len(data):
        raise ValueError("truncated DER")
    tag = data[offset]
    length = data[offset + 1]
    offset += 2
    if length & 0x80:
        count = length & 0x7F
        if count == 0 or offset + count > len(data):
            raise ValueError("bad DER length")
        length = int.from_bytes(data[offset:offset + count], "big")
        offset += count
    if offset + length > len(data):
        raise ValueError("truncated DER value")
    return tag, data[offset:offset + length], offset + length


def _der_ints(value: bytes) -> List[int]:
    """Decode a DER SEQUENCE whose children are INTEGERs."""
    numbers: List[int] = []
    offset = 0
    while offset < len(value):
        tag, child, offset = _der_read(value, offset)
        if tag != 0x02:
            raise ValueError("expected DER INTEGER, got tag 0x%02x" % tag)
        numbers.append(int.from_bytes(child, "big", signed=False))
    return numbers


def _pem_body(text: str, label: str) -> bytes:
    begin = "-----BEGIN %s-----" % label
    end = "-----END %s-----" % label
    start = text.find(begin)
    stop = text.find(end)
    if start < 0 or stop < 0 or stop <= start:
        raise ValueError("PEM block %r not found" % label)
    body = text[start + len(begin):stop]
    return base64.b64decode("".join(body.split()))


def _rsa_numbers_from_pkcs1(der: bytes) -> Tuple[int, int, int]:
    tag, value, _ = _der_read(der, 0)
    if tag != 0x30:
        raise ValueError("PKCS#1 key is not a SEQUENCE")
    numbers = _der_ints(value)
    if len(numbers) < 4:
        raise ValueError("PKCS#1 key has too few integers")
    return numbers[1], numbers[2], numbers[3]


def _rsa_numbers_from_pkcs8(der: bytes) -> Tuple[int, int, int]:
    tag, value, _ = _der_read(der, 0)
    if tag != 0x30:
        raise ValueError("PKCS#8 key is not a SEQUENCE")
    offset = 0
    tag, _child, offset = _der_read(value, offset)
    if tag != 0x02:
        raise ValueError("PKCS#8 version missing")
    tag, algorithm, offset = _der_read(value, offset)
    if tag != 0x30:
        raise ValueError("PKCS#8 algorithm missing")
    oid_tag, oid_value, _ = _der_read(algorithm, 0)
    if oid_tag != 0x06:
        raise ValueError("PKCS#8 algorithm OID missing")
    if oid_value != bytes.fromhex("2a864886f70d010101"):
        raise ValueError("PKCS#8 key is not rsaEncryption")
    tag, private_key, offset = _der_read(value, offset)
    if tag != 0x04:
        raise ValueError("PKCS#8 privateKey OCTET STRING missing")
    return _rsa_numbers_from_pkcs1(private_key)


def rsa_public_numbers_from_der(der: bytes) -> Tuple[int, int]:
    """Parse a DER SubjectPublicKeyInfo (or PKCS#1 RSAPublicKey) -> (n, e)."""
    tag, value, _ = _der_read(der, 0)
    if tag != 0x30:
        raise ValueError("public key is not a SEQUENCE")
    offset = 0
    tag, child, offset = _der_read(value, offset)
    if tag != 0x30:
        raise ValueError("public key algorithm missing")
    oid_tag, oid_value, _ = _der_read(child, 0)
    if oid_tag == 0x06 and oid_value == bytes.fromhex("2a864886f70d010101"):
        tag, bits, offset = _der_read(value, offset)
        if tag != 0x03 or not bits or bits[0] != 0:
            raise ValueError("bad public key BIT STRING")
        tag, rsa_value, _ = _der_read(bits[1:], 0)
        if tag != 0x30:
            raise ValueError("bad RSAPublicKey")
        numbers = _der_ints(rsa_value)
        if len(numbers) < 2:
            raise ValueError("bad RSAPublicKey integers")
        return numbers[0], numbers[1]
    numbers = _der_ints(value)
    if len(numbers) < 2:
        raise ValueError("bad RSAPublicKey integers")
    return numbers[0], numbers[1]


@dataclasses.dataclass
class RsaKey:
    """A 2048-bit RSA key loaded from an adb PEM key file."""

    n: int
    e: int
    d: int
    bits: int = 2048

    @property
    def size_bytes(self) -> int:
        return (self.bits + 7) // 8

    @classmethod
    def from_pem(cls, text: str) -> "RsaKey":
        text = text.strip()
        if "BEGIN RSA PRIVATE KEY" in text:
            der = _pem_body(text, "RSA PRIVATE KEY")
            n, e, d = _rsa_numbers_from_pkcs1(der)
        elif "BEGIN PRIVATE KEY" in text:
            der = _pem_body(text, "PRIVATE KEY")
            n, e, d = _rsa_numbers_from_pkcs8(der)
        else:
            raise AdbKeyError("not a PEM RSA private key")
        bits = n.bit_length()
        if bits < 1024:
            raise AdbKeyError("RSA key too small (%d bits)" % bits)
        return cls(n=n, e=e, d=d, bits=bits)

    @classmethod
    def load(cls, path: str) -> "RsaKey":
        path = os.path.expanduser(path)
        try:
            with open(path, "r", encoding="utf-8") as handle:
                return cls.from_pem(handle.read())
        except OSError as exc:
            raise AdbKeyError("cannot read adb key %s: %s" % (path, exc)) from exc

    @classmethod
    def generate(cls, path: str, bits: int = 2048) -> "RsaKey":
        """Create a new adb key with the openssl CLI (libcrypto is not stdlib)."""
        path = os.path.expanduser(path)
        openssl = shutil.which("openssl")
        if not openssl:
            raise AdbKeyError(
                "no adb key at %s and openssl is not available to generate one; "
                "create it with: openssl genrsa -out %s 2048" % (path, path)
            )
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        tmp_path = path + ".tmp"
        try:
            subprocess.run(
                [openssl, "genrsa", "-out", tmp_path, str(bits)],
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            os.replace(tmp_path, path)
        except (OSError, subprocess.CalledProcessError) as exc:
            raise AdbKeyError("openssl key generation failed: %s" % exc) from exc
        return cls.load(path)

    def sign_sha1(self, data: bytes) -> bytes:
        """RSA PKCS#1 v1.5 signature with SHA-1 (adb AUTH signature)."""
        digest_info = _SHA1_DIGEST_INFO_PREFIX + hashlib.sha1(data).digest()
        k = self.size_bytes
        if len(digest_info) + 11 > k:
            raise AdbKeyError("RSA key too small for SHA-1 DigestInfo")
        padding = b"\xff" * (k - len(digest_info) - 3)
        encoded = b"\x00\x01" + padding + b"\x00" + digest_info
        value = int.from_bytes(encoded, "big")
        return pow(value, self.d, self.n).to_bytes(k, "big")

    def verify_sha1(self, data: bytes, signature: bytes) -> bool:
        """Verify a RSA PKCS#1 v1.5 SHA-1 signature (used by tests/mock)."""
        return verify_sha1(self.n, self.e, data, signature, self.size_bytes)

    def adb_public_key(self) -> str:
        """Android adb public key string (base64 of the 524-byte struct)."""
        return encode_adb_public_key(self.n, self.e, self.size_bytes)

    def public_numbers(self) -> Tuple[int, int]:
        return self.n, self.e


def verify_sha1(n: int, e: int, data: bytes, signature: bytes,
                size_bytes: Optional[int] = None) -> bool:
    """Verify a RSA PKCS#1 v1.5 SHA-1 signature with raw (n, e)."""
    k = size_bytes or ((n.bit_length() + 7) // 8)
    if len(signature) > k:
        return False
    value = pow(int.from_bytes(signature, "big"), e, n)
    encoded = value.to_bytes(k, "big")
    digest_info = _SHA1_DIGEST_INFO_PREFIX + hashlib.sha1(data).digest()
    if not encoded.startswith(b"\x00\x01"):
        return False
    try:
        separator = encoded.index(b"\x00", 2)
    except ValueError:
        return False
    if separator < 10:  # at least 8 bytes of 0xff padding
        return False
    return encoded[separator + 1:] == digest_info


def encode_adb_public_key(n: int, e: int, size_bytes: int = 256) -> str:
    """Encode (n, e) in Android's custom public key format, base64 encoded."""
    if size_bytes != ANDROID_PUBKEY_MODULUS_SIZE:
        raise AdbKeyError(
            "adb AUTH only supports 2048-bit keys (%d-byte modulus), got %d"
            % (ANDROID_PUBKEY_MODULUS_SIZE, size_bytes)
        )
    r32 = 1 << 32
    n0inv = (-pow(n & (r32 - 1), -1, r32)) % r32
    rr = pow(2, 2 * 8 * size_bytes, n)
    blob = (
        struct.pack("<II", ANDROID_PUBKEY_MODULUS_WORDS, n0inv)
        + n.to_bytes(size_bytes, "little")
        + rr.to_bytes(size_bytes, "little")
        + struct.pack("<I", e)
    )
    return base64.b64encode(blob).decode("ascii")


def decode_adb_public_key(text: str) -> Tuple[int, int]:
    """Decode Android's public key encoding (mock/tests) -> (n, e)."""
    text = text.strip().split(" ")[0]
    blob = base64.b64decode(text)
    if len(blob) != ANDROID_PUBKEY_ENCODED_SIZE:
        raise AdbProtocolError(
            "bad android public key length %d (expected %d)"
            % (len(blob), ANDROID_PUBKEY_ENCODED_SIZE)
        )
    words, n0inv = struct.unpack_from("<II", blob, 0)
    if words != ANDROID_PUBKEY_MODULUS_WORDS:
        raise AdbProtocolError("bad android public key modulus words %d" % words)
    offset = 8
    modulus = blob[offset:offset + ANDROID_PUBKEY_MODULUS_SIZE]
    offset += ANDROID_PUBKEY_MODULUS_SIZE
    offset += ANDROID_PUBKEY_MODULUS_SIZE
    (exponent,) = struct.unpack_from("<I", blob, offset)
    return int.from_bytes(modulus, "little"), exponent


def default_key_path() -> str:
    return os.path.expanduser(os.environ.get("ADBKEY") or DEFAULT_KEY_PATH)


def load_or_generate_key(path: Optional[str] = None) -> Tuple["RsaKey", str]:
    """Load the adb private key, generating it with openssl if it is missing."""
    key_path = os.path.expanduser(path or default_key_path())
    if not os.path.exists(key_path):
        LOG.warning("adb key %s not found; generating a new 2048-bit key", key_path)
        key = RsaKey.generate(key_path)
        return key, key_path
    key = RsaKey.load(key_path)
    return key, key_path


def _openssl_sign(key_path: str, data: bytes) -> bytes:
    """Fallback signer: openssl dgst -sha1 -sign (for exotic key formats)."""
    openssl = shutil.which("openssl")
    if not openssl:
        raise AdbKeyError(
            "RSA key could not be parsed in pure Python and openssl is missing"
        )
    with tempfile.NamedTemporaryFile() as tmp:
        tmp.write(data)
        tmp.flush()
        try:
            result = subprocess.run(
                [openssl, "dgst", "-sha1", "-sign", os.path.expanduser(key_path), tmp.name],
                check=True, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            )
        except (OSError, subprocess.CalledProcessError) as exc:
            raise AdbKeyError("openssl signing failed: %s" % exc) from exc
    return result.stdout


# ---------------------------------------------------------------------------
# streams
# ---------------------------------------------------------------------------
@dataclasses.dataclass
class ShellResult:
    """Output of one shell service invocation."""

    stdout: bytes = b""
    stderr: bytes = b""
    exit_code: Optional[int] = None
    protocol: str = "legacy"

    @property
    def output(self) -> bytes:
        return self.stdout + self.stderr


class AdbStream:
    """One adb service stream (OPEN/WRTE/OKAY/CLSE)."""

    def __init__(self, client: "AdbClient", local_id: int, remote_id: int,
                 service: str) -> None:
        self.client = client
        self.local_id = int(local_id)
        self.remote_id = int(remote_id)
        self.service = service
        self._rx = bytearray()
        self._acks = 0
        self.closed = False
        self.closing = False
        self.close_error: Optional[str] = None

    # -- client callbacks ---------------------------------------------------
    def _on_okay(self, packet: AdbPacket) -> None:
        self._acks += 1

    def _on_data(self, packet: AdbPacket) -> None:
        self._rx.extend(packet.data)
        self.client.send_packet(
            AdbPacket(A_OKAY, self.local_id, self.remote_id)
        )

    def _on_close(self, packet: AdbPacket) -> None:
        self.closed = True

    # -- API ----------------------------------------------------------------
    def send(self, data: bytes, *, window: int = 8, timeout: float = 30.0) -> int:
        """Send *data* over the stream with a small pipelined write window."""
        data = bytes(data)
        if self.closed:
            raise AdbStreamClosed("cannot write to closed stream %r" % self.service)
        sent = 0
        chunks = 0
        ack_base = self._acks
        window = max(1, window)
        while sent < len(data):
            while chunks - (self._acks - ack_base) >= window:
                self.client._wait_for(
                    lambda: (chunks - (self._acks - ack_base)) < window or self.closed,
                    timeout, "WRTE ack",
                )
                if self.closed:
                    raise AdbStreamClosed("stream closed while sending %r" % self.service)
            chunk = data[sent:sent + self.client.maxdata]
            self.client.send_packet(
                AdbPacket(A_WRTE, self.local_id, self.remote_id, chunk)
            )
            sent += len(chunk)
            chunks += 1
        return sent

    def read_exact(self, count: int, timeout: float = 30.0) -> bytes:
        """Read exactly *count* bytes from the stream."""
        deadline = time.monotonic() + float(timeout)
        while len(self._rx) < count:
            if self.closed:
                raise AdbStreamClosed(
                    "stream %r closed with %d/%d bytes"
                    % (self.service, len(self._rx), count)
                )
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise vusb.VusbTimeout(
                    "timed out reading %d bytes from stream %r"
                    % (count, self.service)
                )
            self.client._wait_for(
                lambda: len(self._rx) >= count or self.closed, remaining,
                "stream read",
            )
        data = bytes(self._rx[:count])
        del self._rx[:count]
        return data

    def read_available(self, timeout: float = 1.0, max_bytes: Optional[int] = None) -> bytes:
        """Return buffered bytes, waiting up to *timeout* for some to arrive."""
        if not self._rx and not self.closed:
            self.client._wait_for(
                lambda: bool(self._rx) or self.closed, timeout, "stream read",
                allow_timeout=True,
            )
        count = len(self._rx) if max_bytes is None else min(max_bytes, len(self._rx))
        data = bytes(self._rx[:count])
        del self._rx[:count]
        return data

    def read_all(self, *, timeout: float = 60.0, max_bytes: Optional[int] = None) -> bytes:
        """Read until the device closes the stream (or *max_bytes* arrived)."""
        deadline = time.monotonic() + float(timeout)
        while not self.closed:
            if max_bytes is not None and len(self._rx) >= max_bytes:
                break
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise vusb.VusbTimeout("timed out reading stream %r" % self.service)
            self.client._wait_for(
                lambda: self.closed, remaining, "stream close",
            )
        data = bytes(self._rx)
        self._rx.clear()
        return data

    def close(self, *, timeout: float = 2.0) -> None:
        """Send CLSE and wait for the device to close its side."""
        if self.closed or self.closing:
            return
        self.closing = True
        try:
            self.client.send_packet(AdbPacket(A_CLSE, self.local_id, self.remote_id))
        except vusb.VusbError:
            self.closed = True
            return
        try:
            self.client._wait_for(lambda: self.closed, timeout, "stream close",
                                  allow_timeout=True)
        finally:
            self.closed = True


# ---------------------------------------------------------------------------
# sideload
# ---------------------------------------------------------------------------
@dataclasses.dataclass
class SideloadStats:
    """Result of one sideload transfer."""

    file_size: int
    bytes_served: int
    blocks: int
    seconds: float
    protocol: str = "sideload-host"
    ok: bool = True

    @property
    def percent(self) -> float:
        if self.file_size <= 0:
            return 100.0
        # AOSP estimates completion as xfer * 47 / size because a package is
        # typically read twice (verify + install) plus zip overhead, so 100%
        # corresponds to about 2.13x the package size.
        return min(100.0, 100.0 * 47.0 * self.bytes_served / (self.file_size * 100.0))

    def __str__(self) -> str:  # pragma: no cover - cosmetic
        return "sideload %s: %d/%d bytes, %d blocks in %.2fs" % (
            self.protocol, self.bytes_served, self.file_size,
            self.blocks, self.seconds,
        )


SideloadProgress = Callable[[SideloadStats], None]


# ---------------------------------------------------------------------------
# client
# ---------------------------------------------------------------------------
class AdbClient:
    """adb client over the virtual UDC bulk transport.

    The default transport is the same framed chardev used by fastboot; pass a
    custom VUsbHost or subclass and override _transport_send / _transport_recv
    once the kernel-gadget model exposes different endpoints.
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
        key_path: Optional[str] = None,
        handshake_timeout: float = 10.0,
        auth_timeout: float = 10.0,
    ) -> None:
        self.usb = usb if usb is not None else vusb.VUsbHost(
            host, port, connect_timeout=connect_timeout, io_timeout=io_timeout
        )
        self.host = host
        self.port = int(port)
        self.maxdata = int(maxdata)
        self.banner_text = banner
        self.soft_conn_timeout = soft_conn_timeout
        self.pullup_settle = pullup_settle
        self.bus_settle = bus_settle
        self.qmu_timeout = qmu_timeout
        self.response_idle = response_idle
        self.key_path = os.path.expanduser(key_path) if key_path else None
        self.handshake_timeout = handshake_timeout
        self.auth_timeout = auth_timeout

        self.device: Optional[vusb.UsbDeviceInfo] = None
        self.banner: Optional[AdbBanner] = None
        self.features: set = set()
        self.key: Optional[RsaKey] = None
        self.auth_used: Optional[str] = None
        self.streams: Dict[int, AdbStream] = {}
        self._reader = AdbPacketReader()
        self._pending: List[AdbPacket] = []
        self._next_local_id = 1

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

    # -- transport abstraction ---------------------------------------------
    def _transport_send(self, data: bytes) -> None:
        self.usb.bulk_out_raw(data)

    def _transport_recv(self, timeout: float) -> bytes:
        return self.usb.bulk_in(timeout=timeout, idle_gap=self.response_idle)

    # -- packet I/O ---------------------------------------------------------
    def send_packet(self, packet: AdbPacket) -> None:
        LOG.debug("-> %s", packet)
        self._transport_send(packet.encode())

    def recv_packet(self, timeout: float = 5.0) -> AdbPacket:
        deadline = time.monotonic() + float(timeout)
        while True:
            if self._pending:
                return self._pending.pop(0)
            packet = self._reader.pop_packet()
            if packet is not None:
                LOG.debug("<- %s", packet)
                return packet
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise vusb.VusbTimeout("timed out waiting for an adb packet")
            raw = self._transport_recv(remaining)
            self._reader.feed(raw)

    # -- internal event pump ------------------------------------------------
    def _pump_one(self, timeout: float) -> None:
        packet = self.recv_packet(timeout)
        self._dispatch(packet)

    def _dispatch(self, packet: AdbPacket) -> None:
        command = packet.command
        if command == A_OKAY:
            stream = self.streams.get(packet.arg1)
            if stream is not None:
                stream._on_okay(packet)
            return
        if command == A_WRTE:
            stream = self.streams.get(packet.arg1)
            if stream is not None:
                stream._on_data(packet)
            else:
                LOG.warning("WRTE for unknown stream %d", packet.arg1)
                self.send_packet(AdbPacket(A_CLSE, packet.arg1, packet.arg0))
            return
        if command == A_CLSE:
            stream = self.streams.get(packet.arg1)
            if stream is not None:
                stream._on_close(packet)
            return
        if command == A_OPEN:
            LOG.warning("device-initiated OPEN ignored: %r", packet.data[:64])
            self.send_packet(AdbPacket(A_CLSE, packet.arg1, packet.arg0))
            return
        LOG.debug("unhandled packet: %s", packet)

    def _wait_for(self, predicate, timeout: float, what: str,
                  *, allow_timeout: bool = False) -> bool:
        """Pump events until *predicate()* is true; optionally tolerate timeout."""
        deadline = time.monotonic() + float(timeout)
        while True:
            if predicate():
                return True
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                if allow_timeout:
                    return False
                raise vusb.VusbTimeout("timed out waiting for %s" % what)
            try:
                self._pump_one(min(remaining, max(0.05, self.response_idle)))
            except vusb.VusbTimeout:
                continue

    # -- handshake ----------------------------------------------------------
    def cnxn(self, *, timeout: float = 5.0) -> AdbPacket:
        """Send CNXN and return the first device packet (CNXN or AUTH)."""
        self.send_packet(
            AdbPacket(A_CNXN, A_VERSION, self.maxdata, self.banner_text.encode("ascii"))
        )
        return self.recv_packet(timeout=timeout)

    def handshake(self, *, timeout: Optional[float] = None,
                  auth_timeout: Optional[float] = None) -> AdbBanner:
        """CNXN + optional RSA AUTH; returns the parsed device banner."""
        limit = self.handshake_timeout if timeout is None else float(timeout)
        auth_limit = self.auth_timeout if auth_timeout is None else float(auth_timeout)
        self.send_packet(
            AdbPacket(A_CNXN, A_VERSION, self.maxdata, self.banner_text.encode("ascii"))
        )
        signed = False
        sent_public_key = False
        deadline = time.monotonic() + limit
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise AdbAuthError("timed out during adb handshake")
            packet = self.recv_packet(timeout=remaining)
            if packet.command == A_CNXN:
                self.banner = AdbBanner.parse(packet.data, packet.arg0, packet.arg1)
                self.maxdata = max(256, min(self.maxdata, self.banner.maxdata or self.maxdata))
                self.features = set(self.banner.features)
                if self.auth_used is None:
                    self.auth_used = "none"
                LOG.info("adb handshake ok: state=%s device=%s maxdata=%d features=%s",
                         self.banner.state, self.banner.device_name,
                         self.maxdata, ",".join(sorted(self.features)))
                return self.banner
            if packet.command == A_AUTH:
                if packet.arg0 == AUTH_TOKEN:
                    token = packet.data
                    if not signed:
                        signature = self._sign_token(token)
                        self.auth_used = "signature"
                        self.send_packet(AdbPacket(A_AUTH, AUTH_SIGNATURE, 0, signature))
                        signed = True
                        LOG.info("sent AUTH signature for token %s", token.hex())
                    elif not sent_public_key:
                        key = self._ensure_key()
                        public_key = key.adb_public_key()
                        user = os.environ.get("USER") or "user"
                        host = os.uname().nodename if hasattr(os, "uname") else "host"
                        payload = (public_key + " " + user + "@" + host + "\x00").encode("latin-1")
                        self.auth_used = "publickey"
                        self.send_packet(AdbPacket(A_AUTH, AUTH_RSAPUBLICKEY, 0, payload))
                        sent_public_key = True
                        LOG.info("sent AUTH public key upload (%d bytes)", len(payload))
                    else:
                        raise AdbAuthError(
                            "device rejected both AUTH signature and public key; "
                            "check adb authorization on the device"
                        )
                    continue
                if packet.arg0 == AUTH_RSAPUBLICKEY:
                    continue
                LOG.warning("unexpected AUTH kind %d", packet.arg0)
                continue
            if packet.command == A_CLSE:
                raise AdbAuthError("device closed the connection during handshake")
            raise AdbProtocolError("unexpected packet during handshake: %s" % packet)

    def _sign_token(self, token: bytes) -> bytes:
        key, path = self._load_key()
        try:
            return key.sign_sha1(token)
        except AdbKeyError:
            LOG.warning("falling back to openssl signing for %s", path)
            return _openssl_sign(path, token)

    def _ensure_key(self) -> RsaKey:
        return self._load_key()[0]

    def _load_key(self) -> Tuple[RsaKey, str]:
        if self.key is None:
            self.key, path = load_or_generate_key(self.key_path)
            self.key_path = path
        return self.key, self.key_path or default_key_path()

    # -- streams ------------------------------------------------------------
    def open_stream(self, service: str, *, timeout: float = 10.0) -> AdbStream:
        """Open an adb service and return the connected stream."""
        local_id = self._next_local_id
        self._next_local_id += 1
        self.send_packet(
            AdbPacket(A_OPEN, local_id, 0, service.encode("utf-8") + b"\x00")
        )
        deadline = time.monotonic() + float(timeout)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise vusb.VusbTimeout("timed out opening service %r" % service)
            packet = self.recv_packet(timeout=remaining)
            if packet.command == A_OKAY and packet.arg1 == local_id:
                stream = AdbStream(self, local_id, packet.arg0, service)
                self.streams[local_id] = stream
                LOG.info("opened %r (local=%d remote=%d)", service, local_id, packet.arg0)
                return stream
            if packet.command == A_CLSE and packet.arg1 == local_id:
                raise AdbStreamError("device refused service %r (closed)" % service)
            LOG.debug("pending packet before OPEN ok: %s", packet)

    def open(self, service: str, *, timeout: float = 5.0) -> Tuple[int, int]:
        """Compatibility helper: open a service, return (local_id, remote_id)."""
        stream = self.open_stream(service, timeout=timeout)
        return stream.local_id, stream.remote_id

    def write(self, local_id: int, remote_id: int, data: bytes,
              *, timeout: float = 5.0) -> int:
        """Compatibility helper: send data on a raw stream id pair."""
        stream = self.streams.get(local_id)
        if stream is None:
            stream = AdbStream(self, local_id, remote_id, "<raw>")
        stream.send(data, timeout=timeout)
        return local_id

    def close_stream(self, local_id: int, remote_id: int) -> None:
        stream = self.streams.get(local_id)
        if stream is not None:
            stream.close()
        else:
            self.send_packet(AdbPacket(A_CLSE, local_id, remote_id))

    # -- high level services ------------------------------------------------
    def device_label(self) -> str:
        return self.banner.serial if self.banner else "unknown"

    def device_state(self) -> str:
        return self.banner.state if self.banner else "device"

    def shell(self, command: str, *, timeout: float = 30.0,
              protocol: str = "auto") -> ShellResult:
        """Run a command via the adbd shell service (v2 when available)."""
        use_v2 = protocol == "v2" or (
            protocol == "auto" and "shell_v2" in self.features
        )
        if use_v2:
            service = "shell,v2,raw:" + command
            stream = self.open_stream(service, timeout=timeout)
            try:
                stdout = bytearray()
                stderr = bytearray()
                exit_code: Optional[int] = None
                while True:
                    try:
                        header = stream.read_exact(5, timeout=timeout)
                    except AdbStreamClosed:
                        break
                    stream_id = header[0]
                    length = int.from_bytes(header[1:5], "little")
                    payload = stream.read_exact(length, timeout=timeout) if length else b""
                    if stream_id == 1:
                        stdout.extend(payload)
                    elif stream_id == 2:
                        stderr.extend(payload)
                    elif stream_id == 3:
                        if payload:
                            value = payload[0]
                            exit_code = value if value < 128 else value - 256
                        break
                return ShellResult(bytes(stdout), bytes(stderr), exit_code, "v2")
            finally:
                stream.close()
        service = "shell:" + command
        stream = self.open_stream(service, timeout=timeout)
        try:
            output = stream.read_all(timeout=timeout)
            return ShellResult(output, b"", None, "legacy")
        finally:
            stream.close()

    def reboot(self, target: str = "", *, timeout: float = 30.0) -> str:
        """Reboot the device: target in {..., bootloader, recovery, fastboot, rescue}."""
        valid = {"", "bootloader", "recovery", "fastboot", "rescue"}
        if target not in valid:
            raise ValueError("bad reboot target %r" % (target,))
        service = "reboot:" + target
        stream = self.open_stream(service, timeout=timeout)
        try:
            stream.read_all(timeout=timeout)
        except (AdbStreamClosed, vusb.VusbTimeout):
            pass
        finally:
            try:
                stream.close(timeout=1.0)
            except vusb.VusbError:
                pass
        return service

    # -- sideload -----------------------------------------------------------
    def sideload(self, path: str, *, block_size: int = SIDELOAD_BLOCK_SIZE,
                 progress: Optional[SideloadProgress] = None,
                 timeout: float = 120.0,
                 legacy_fallback: bool = True) -> SideloadStats:
        """Sideload an OTA package using recovery's minadbd services.

        Modern minadbd (LineageOS/AOSP recovery) refuses the legacy push
        service and expects sideload-host:<size>:<block>: the device asks for
        block indices and the host serves them from the package file.
        """
        path = os.path.expanduser(path)
        try:
            file_size = os.path.getsize(path)
        except OSError as exc:
            raise SideloadError("cannot stat package %s: %s" % (path, exc)) from exc
        if file_size <= 0:
            raise SideloadError("package %s is empty" % path)
        LOG.info("sideloading %s (%d bytes, block=%d)", path, file_size, block_size)
        try:
            stream = self.open_stream(
                "sideload-host:%d:%d" % (file_size, block_size), timeout=timeout
            )
        except (AdbError, vusb.VusbError) as exc:
            if not legacy_fallback or file_size > 0x7FFFFFFF:
                raise SideloadError(
                    "device does not support sideload-host and legacy fallback is "
                    "unavailable: %s" % exc
                ) from exc
            LOG.warning("sideload-host refused (%s); trying legacy sideload", exc)
            return self._sideload_legacy(path, file_size, progress, timeout)
        try:
            return self._serve_sideload_host(
                stream, path, file_size, block_size, progress, timeout
            )
        finally:
            stream.close(timeout=3.0)

    def _serve_sideload_host(self, stream: AdbStream, path: str, file_size: int,
                             block_size: int, progress: Optional[SideloadProgress],
                             timeout: float) -> SideloadStats:
        started = time.monotonic()
        served = 0
        blocks = 0
        ok = True
        with open(path, "rb") as package:
            while True:
                command = stream.read_exact(8, timeout=timeout)
                if command == SIDELOAD_EXIT_SUCCESS:
                    ok = True
                    break
                if command == SIDELOAD_EXIT_FAILURE:
                    ok = False
                    break
                try:
                    block = int(command.decode("ascii"))
                except (UnicodeDecodeError, ValueError) as exc:
                    raise SideloadError(
                        "device sent a bad block request %r" % command
                    ) from exc
                offset = block * block_size
                if offset >= file_size:
                    raise SideloadError(
                        "device requested block %d past end of %d-byte package"
                        % (block, file_size)
                    )
                count = min(block_size, file_size - offset)
                package.seek(offset)
                data = package.read(count)
                if len(data) != count:
                    raise SideloadError("short read from package at offset %d" % offset)
                stream.send(data)
                served += count
                blocks += 1
                if progress is not None:
                    progress(SideloadStats(
                        file_size=file_size, bytes_served=served, blocks=blocks,
                        seconds=time.monotonic() - started,
                        protocol="sideload-host", ok=True,
                    ))
        stats = SideloadStats(
            file_size=file_size, bytes_served=served, blocks=blocks,
            seconds=time.monotonic() - started, protocol="sideload-host", ok=ok,
        )
        if not ok:
            raise SideloadError(
                "device reported FAILFAIL after %d bytes (%d blocks)"
                % (served, blocks)
            )
        LOG.info("sideload-host done: %s", stats)
        return stats

    def _sideload_legacy(self, path: str, file_size: int,
                         progress: Optional[SideloadProgress],
                         timeout: float) -> SideloadStats:
        started = time.monotonic()
        stream = self.open_stream("sideload:%d" % file_size, timeout=timeout)
        served = 0
        blocks = 0
        try:
            with open(path, "rb") as package:
                while True:
                    chunk = package.read(SIDELOAD_BLOCK_SIZE)
                    if not chunk:
                        break
                    stream.send(chunk)
                    served += len(chunk)
                    blocks += 1
                    if progress is not None:
                        progress(SideloadStats(
                            file_size=file_size, bytes_served=served, blocks=blocks,
                            seconds=time.monotonic() - started,
                            protocol="sideload", ok=True,
                        ))
            status = stream.read_available(timeout=1.0)
            if status.startswith(b"FAIL"):
                raise SideloadError(
                    "legacy sideload failed: %s" % status.decode("latin-1", "replace")
                )
        finally:
            stream.close(timeout=3.0)
        stats = SideloadStats(file_size=file_size, bytes_served=served, blocks=blocks,
                              seconds=time.monotonic() - started,
                              protocol="sideload", ok=True)
        LOG.info("legacy sideload done: %s", stats)
        return stats

    # -- optional sync push -------------------------------------------------
    def push(self, local_path: str, remote_path: str, *,
             timeout: float = 120.0,
             progress: Optional[Callable[[int, int], None]] = None) -> int:
        """Push one file with the adb sync SEND sub-protocol."""
        local_path = os.path.expanduser(local_path)
        with open(local_path, "rb") as handle:
            data = handle.read()
        stream = self.open_stream("sync:", timeout=timeout)
        try:
            self._sync_send(stream, remote_path, data, timeout, progress)
        finally:
            stream.close(timeout=3.0)
        return len(data)

    def _sync_send(self, stream: AdbStream, remote_path: str, data: bytes,
                   timeout: float, progress) -> None:
        def request(payload: bytes) -> None:
            stream.send(payload, timeout=timeout)

        def read_response() -> Tuple[str, bytes]:
            head = stream.read_exact(8, timeout=timeout)
            return head[:4].decode("ascii"), head[4:]

        name = remote_path.encode("utf-8")
        request(b"SEND" + struct.pack("<I", len(name)) + name)
        kind, payload = read_response()
        if kind == "FAIL":
            raise AdbStreamError("push refused: %s" % payload.decode("latin-1", "replace"))
        offset = 0
        while offset < len(data):
            chunk = data[offset:offset + SIDELOAD_BLOCK_SIZE]
            request(b"DATA" + struct.pack("<I", len(chunk)) + chunk)
            offset += len(chunk)
            if progress is not None:
                progress(offset, len(data))
        mtime = int(time.time())
        request(b"DONE" + struct.pack("<I", mtime))
        kind, payload = read_response()
        if kind != "OKAY":
            raise AdbStreamError("push failed: %s %r" % (kind, payload[:32]))
