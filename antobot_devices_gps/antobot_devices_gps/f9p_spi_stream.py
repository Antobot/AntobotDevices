"""Bounded mixed NMEA/UBX frame reading for the F9P SPI stream."""

from dataclasses import dataclass
import math
import statistics
import struct
from typing import Callable, List, Optional, Union


UBX_NAV_CLASS = 0x01
UBX_NAV_SIG_ID = 0x43
UBX_NAV_SIG_CARRIER_USED = 1 << 4


@dataclass(frozen=True)
class NmeaFrame:
    """A complete NMEA sentence, including its line ending."""

    sentence: str


@dataclass(frozen=True)
class UbxFrame:
    """A checksummed UBX message."""

    message_class: int
    message_id: int
    payload: bytes


StreamFrame = Union[NmeaFrame, UbxFrame]


class F9PSpiFrameReader:
    """Read one complete NMEA or UBX frame without oversized SPI transfers.

    The reader searches for a frame start byte-by-byte.  Only after a UBX
    header declares its payload length does it use bounded multi-byte reads.
    This avoids polling arbitrary 512-byte blocks while reducing a NAV-SIG
    message from more than a thousand SPI transfers to small 32-byte chunks.
    """

    def __init__(
        self,
        read: Callable[[int], bytes],
        max_nmea_bytes: int = 256,
        max_ubx_payload_bytes: int = 2048,
        ubx_chunk_size: int = 32,
        idle_reads_before_return: int = 8,
    ) -> None:
        if max_nmea_bytes < 8:
            raise ValueError("max_nmea_bytes must be at least 8")
        if max_ubx_payload_bytes < 0:
            raise ValueError("max_ubx_payload_bytes must be non-negative")
        if not 1 <= ubx_chunk_size <= 64:
            raise ValueError("ubx_chunk_size must be in [1, 64]")
        if idle_reads_before_return <= 0:
            raise ValueError("idle_reads_before_return must be positive")

        self._read = read
        self.max_nmea_bytes = max_nmea_bytes
        self.max_ubx_payload_bytes = max_ubx_payload_bytes
        self.ubx_chunk_size = ubx_chunk_size
        self.idle_reads_before_return = idle_reads_before_return
        self._nmea_buffer = bytearray()
        self._ubx_buffer = bytearray()

    def read_frame(self) -> Optional[StreamFrame]:
        """Return the next complete frame, or None when SPI is idle."""
        if self._nmea_buffer:
            return self._read_nmea()
        if self._ubx_buffer:
            return self._read_ubx()

        idle_reads = 0
        while idle_reads < self.idle_reads_before_return:
            first = self._read_one()
            if first is None:
                idle_reads += 1
                continue
            idle_reads = 0

            if first == ord("$"):
                self._nmea_buffer = bytearray(b"$")
                return self._read_nmea()
            if first != 0xB5:
                continue

            self._ubx_buffer = bytearray(b"\xb5")
            return self._read_ubx()

        return None

    def _read_one(self) -> Optional[int]:
        data = self._read(1)
        if len(data) != 1 or data == b"\xff":
            return None
        return data[0]

    def _read_nmea(self) -> Optional[NmeaFrame]:
        idle_reads = 0
        while len(self._nmea_buffer) < self.max_nmea_bytes:
            value = self._read_one()
            if value is None:
                idle_reads += 1
                if idle_reads >= self.idle_reads_before_return:
                    return None
                continue
            idle_reads = 0
            self._nmea_buffer.append(value)
            if value == ord("\n"):
                sentence = self._nmea_buffer.decode("ascii", errors="replace")
                self._nmea_buffer.clear()
                return NmeaFrame(sentence)
        self._nmea_buffer.clear()
        return None

    def _read_ubx(self) -> Optional[UbxFrame]:
        idle_reads = 0
        while len(self._ubx_buffer) < 2:
            value = self._read_one()
            if value is None:
                idle_reads += 1
                if idle_reads >= self.idle_reads_before_return:
                    return None
                continue
            if len(self._ubx_buffer) == 1 and value != 0x62:
                self._ubx_buffer.clear()
                if value == ord("$"):
                    self._nmea_buffer = bytearray(b"$")
                    return self._read_nmea()
                return None
            self._ubx_buffer.append(value)
            idle_reads = 0

        while len(self._ubx_buffer) < 6:
            value = self._read_one()
            if value is None:
                idle_reads += 1
                if idle_reads >= self.idle_reads_before_return:
                    return None
                continue
            self._ubx_buffer.append(value)
            idle_reads = 0

        payload_length = self._ubx_buffer[4] | (self._ubx_buffer[5] << 8)
        if payload_length > self.max_ubx_payload_bytes:
            self._ubx_buffer.clear()
            return None

        frame_length = 8 + payload_length
        while len(self._ubx_buffer) < frame_length:
            request_size = min(self.ubx_chunk_size, frame_length - len(self._ubx_buffer))
            chunk = self._read(request_size)
            if len(chunk) != request_size:
                return None
            # Once the frame length is known, 0xFF is legal payload/checksum
            # data and must not be interpreted as the SPI idle value.
            self._ubx_buffer.extend(chunk)

        raw_frame = bytes(self._ubx_buffer)
        self._ubx_buffer.clear()
        if _ubx_checksum(raw_frame[2:-2]) != raw_frame[-2:]:
            return None
        return UbxFrame(raw_frame[2], raw_frame[3], raw_frame[6:-2])


def _ubx_checksum(data: bytes) -> bytes:
    check_a = 0
    check_b = 0
    for value in data:
        check_a = (check_a + value) & 0xFF
        check_b = (check_b + check_a) & 0xFF
    return bytes((check_a, check_b))


@dataclass(frozen=True)
class NavSigSignal:
    gnss_id: int
    sv_id: int
    sig_id: int
    freq_id: int
    cno_dbhz: int
    quality_ind: int
    corr_source: int
    sig_flags: int

    @property
    def carrier_used(self) -> bool:
        return bool(self.sig_flags & UBX_NAV_SIG_CARRIER_USED)

    @property
    def unhealthy(self) -> bool:
        return (self.sig_flags & 0x03) == 2


@dataclass(frozen=True)
class NavSigReport:
    itow_ms: int
    signals: List[NavSigSignal]


@dataclass(frozen=True)
class NavSigHealth:
    valid: bool
    satellites_in_view: int
    constellation_count: int
    weak_signal_count: int
    cno_mean_dbhz: float
    cno_median_dbhz: float
    cno_p10_dbhz: float


def parse_nav_sig(payload: bytes) -> NavSigReport:
    """Decode the protocol-version-27 UBX-NAV-SIG payload."""
    if len(payload) < 8:
        raise ValueError("NAV-SIG payload is shorter than its header")

    itow_ms, version, signal_count, _reserved = struct.unpack_from("<IBBH", payload)
    if version != 0:
        raise ValueError("unsupported NAV-SIG version %d" % version)
    expected_length = 8 + 16 * signal_count
    if len(payload) != expected_length:
        raise ValueError(
            "NAV-SIG payload length %d does not match %d signals"
            % (len(payload), signal_count)
        )

    signals = []
    for index in range(signal_count):
        offset = 8 + 16 * index
        (
            gnss_id,
            sv_id,
            sig_id,
            freq_id,
            _pr_res,
            cno_dbhz,
            quality_ind,
            corr_source,
            _iono_model,
            sig_flags,
            _reserved,
        ) = struct.unpack_from("<BBBBhBBBBHI", payload, offset)
        signals.append(
            NavSigSignal(
                gnss_id=gnss_id,
                sv_id=sv_id,
                sig_id=sig_id,
                freq_id=freq_id,
                cno_dbhz=cno_dbhz,
                quality_ind=quality_ind,
                corr_source=corr_source,
                sig_flags=sig_flags,
            )
        )
    return NavSigReport(itow_ms=itow_ms, signals=signals)


def nav_sig_health(report: NavSigReport, weak_cno_dbhz: float = 25.0) -> NavSigHealth:
    """Summarize C/N0, preferring carrier signals used by the solution."""
    tracked = [
        signal
        for signal in report.signals
        if signal.quality_ind >= 5
        and not signal.unhealthy
        and signal.cno_dbhz > 0
    ]
    selected = [signal for signal in tracked if signal.carrier_used] or tracked
    if not selected:
        return NavSigHealth(False, 0, 0, 0, math.nan, math.nan, math.nan)

    values = sorted(signal.cno_dbhz for signal in selected)
    p10_index = max(0, math.ceil(len(values) * 0.10) - 1)
    return NavSigHealth(
        valid=True,
        satellites_in_view=len({(signal.gnss_id, signal.sv_id) for signal in selected}),
        constellation_count=len({signal.gnss_id for signal in selected}),
        weak_signal_count=sum(value < weak_cno_dbhz for value in values),
        cno_mean_dbhz=statistics.fmean(values),
        cno_median_dbhz=float(statistics.median(values)),
        cno_p10_dbhz=float(values[p10_index]),
    )
