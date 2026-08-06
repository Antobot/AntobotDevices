#!/usr/bin/env python3
"""Echo raw F9P SPI NMEA data and decode GGA fixes for uRCU debugging."""

import argparse
from dataclasses import dataclass
import logging
import math
import statistics
import struct
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Union

import pynmea2
import spidev

# Allow running the script directly from the source tree.  A copied script
# requires the package to be available through `source install/setup.bash`.
PACKAGE_SOURCE_ROOT = Path(__file__).resolve().parents[2]
if str(PACKAGE_SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SOURCE_ROOT))

from antobot_devices_gps.ublox_gps.ublox_gps import UbloxGps


QUALITY_NAMES = {
    0: "Invalid",
    1: "GPS",
    2: "DGPS",
    4: "RTK Fixed",
    5: "RTK Float",
    6: "Estimated",
}

UBX_NAV_CLASS = 0x01
UBX_NAV_SIG_ID = 0x43
UBX_NAV_SIG_CARRIER_USED = 1 << 4


@dataclass(frozen=True)
class NmeaFrame:
    """One complete NMEA sentence extracted from the mixed F9P stream."""

    sentence: str


@dataclass(frozen=True)
class UbxFrame:
    """One checksummed UBX frame extracted from the mixed F9P stream."""

    message_class: int
    message_id: int
    payload: bytes


StreamFrame = Union[NmeaFrame, UbxFrame]


@dataclass(frozen=True)
class NavSigSignal:
    """The signal-level fields used for C/N0 health evaluation."""

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
    """Decoded UBX-NAV-SIG epoch."""

    itow_ms: int
    signals: List[NavSigSignal]


class MixedF9PFrameDecoder:
    """Incrementally extract NMEA and UBX frames from one byte stream."""

    def __init__(self, max_nmea_bytes: int = 256, max_ubx_payload_bytes: int = 2048) -> None:
        self.buffer = bytearray()
        self.max_nmea_bytes = max_nmea_bytes
        self.max_ubx_payload_bytes = max_ubx_payload_bytes

    @property
    def is_mid_ubx_frame(self) -> bool:
        """True once a UBX header has been received and its payload is pending."""
        return len(self.buffer) >= 2 and self.buffer[:2] == b"\xb5\x62"

    def feed(self, data: bytes) -> List[StreamFrame]:
        """Add stream bytes and return every complete, checksummed frame."""
        self.buffer.extend(data)
        frames: List[StreamFrame] = []

        while self.buffer:
            start = self._first_frame_start()
            if start is None:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1] == 0xB5 else b""
                break
            if start:
                del self.buffer[:start]

            if self.buffer[0] == ord("$"):
                frame = self._pop_nmea()
                if frame is None:
                    break
                frames.append(frame)
                continue

            frame = self._pop_ubx()
            if frame is None:
                break
            if frame is False:
                del self.buffer[0]
                continue
            frames.append(frame)

        return frames

    def _first_frame_start(self) -> Optional[int]:
        nmea_start = self.buffer.find(b"$")
        ubx_start = self.buffer.find(b"\xb5")
        starts = [index for index in (nmea_start, ubx_start) if index >= 0]
        return min(starts) if starts else None

    def _pop_nmea(self) -> Optional[NmeaFrame]:
        newline = self.buffer.find(b"\n")
        if newline < 0:
            if len(self.buffer) > self.max_nmea_bytes:
                del self.buffer[0]
            return None

        raw_frame = bytes(self.buffer[: newline + 1])
        del self.buffer[: newline + 1]
        return NmeaFrame(raw_frame.decode("ascii", errors="replace"))

    def _pop_ubx(self) -> Union[UbxFrame, bool, None]:
        if len(self.buffer) < 2:
            return None
        if self.buffer[1] != 0x62:
            return False
        if len(self.buffer) < 6:
            return None

        payload_length = self.buffer[4] | (self.buffer[5] << 8)
        if payload_length > self.max_ubx_payload_bytes:
            return False
        frame_length = 8 + payload_length
        if len(self.buffer) < frame_length:
            return None

        raw_frame = bytes(self.buffer[:frame_length])
        if _ubx_checksum(raw_frame[2:-2]) != raw_frame[-2:]:
            return False

        del self.buffer[:frame_length]
        return UbxFrame(raw_frame[2], raw_frame[3], raw_frame[6:-2])


def _ubx_checksum(data: bytes) -> bytes:
    check_a = 0
    check_b = 0
    for value in data:
        check_a = (check_a + value) & 0xFF
        check_b = (check_b + check_a) & 0xFF
    return bytes((check_a, check_b))


def parse_nav_sig(payload: bytes) -> NavSigReport:
    """Decode a protocol-version-27 UBX-NAV-SIG payload."""
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


def _as_float(value: object) -> Optional[float]:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _as_int(value: object) -> Optional[int]:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def is_gga(raw_line: str) -> bool:
    """Return whether an NMEA-looking sentence declares the GGA type."""
    nmea_start = raw_line.find("$")
    return raw_line[nmea_start + 3 : nmea_start + 6] == "GGA"


def has_valid_nmea_checksum(raw_line: str) -> bool:
    """Validate the mandatory NMEA checksum before accessing parsed fields."""
    nmea_start = raw_line.find("$")
    if nmea_start < 0:
        return False

    sentence = raw_line[nmea_start:].strip()
    checksum_start = sentence.find("*")
    if checksum_start < 1 or len(sentence) != checksum_start + 3:
        return False

    try:
        expected_checksum = int(sentence[checksum_start + 1 :], 16)
    except ValueError:
        return False

    calculated_checksum = 0
    for character in sentence[1:checksum_start]:
        calculated_checksum ^= ord(character)
    return calculated_checksum == expected_checksum


def parse_gga(raw_line: str) -> Optional[Dict[str, object]]:
    """Parse one NMEA GGA sentence, returning None for all other input."""
    nmea_start = raw_line.find("$")
    if nmea_start < 0 or not has_valid_nmea_checksum(raw_line):
        return None

    try:
        sentence = pynmea2.parse(raw_line[nmea_start:].strip())
        if sentence.sentence_type != "GGA":
            return None

        quality = _as_int(sentence.gps_qual)
        return {
            "utc": str(sentence.timestamp),
            "latitude": _as_float(sentence.latitude),
            "longitude": _as_float(sentence.longitude),
            "altitude_m": _as_float(sentence.altitude),
            "quality": quality,
            "quality_name": QUALITY_NAMES.get(quality, "Unknown"),
            "satellites": _as_int(sentence.num_sats),
            "hdop": _as_float(sentence.horizontal_dil),
            "correction_age_s": _as_float(sentence.age_gps_data),
        }
    except (AttributeError, pynmea2.ParseError, TypeError, ValueError):
        return None


def nmea_callback(raw_line: str, raw_all: bool) -> None:
    """Log a raw NMEA line and its GGA fields when present."""
    gga_sentence = is_gga(raw_line)
    checksum_valid = has_valid_nmea_checksum(raw_line)
    if raw_all or gga_sentence:
        logging.info("RAW: %s", raw_line.rstrip())

    if not checksum_valid:
        logging.warning("DROP invalid NMEA checksum")
        return

    if not gga_sentence:
        return

    gga = parse_gga(raw_line)
    if gga is None:
        logging.warning("DROP invalid GGA fields")
        return

    logging.info(
        "GGA fix: utc=%s lat=%.8f lon=%.8f alt_m=%s quality=%s(%s) "
        "sats=%s hdop=%s correction_age_s=%s",
        gga["utc"],
        gga["latitude"] if gga["latitude"] is not None else float("nan"),
        gga["longitude"] if gga["longitude"] is not None else float("nan"),
        gga["altitude_m"],
        gga["quality"],
        gga["quality_name"],
        gga["satellites"],
        gga["hdop"],
        gga["correction_age_s"],
    )


def nav_sig_callback(report: NavSigReport, show_details: bool) -> None:
    """Log C/N0 statistics for carrier signals actually used in the solution."""
    selected = [
        signal
        for signal in report.signals
        if signal.quality_ind >= 5
        and signal.carrier_used
        and not signal.unhealthy
        and signal.cno_dbhz > 0
    ]
    logging.info(
        "NAV-SIG: itow_ms=%d tracked=%d carrier_used=%d",
        report.itow_ms,
        len(report.signals),
        len(selected),
    )
    if not selected:
        logging.warning("NAV-SIG has no healthy carrier signal used by the navigation solution")
        return

    cno_values = sorted(signal.cno_dbhz for signal in selected)
    p10_index = max(0, math.ceil(len(cno_values) * 0.10) - 1)
    logging.info(
        "NAV-SIG C/N0: mean=%.1f median=%.1f p10=%d weak_lt25=%d unique_sv=%d",
        statistics.fmean(cno_values),
        statistics.median(cno_values),
        cno_values[p10_index],
        sum(value < 25 for value in cno_values),
        len({(signal.gnss_id, signal.sv_id) for signal in selected}),
    )
    if show_details:
        for signal in selected:
            logging.info(
                "NAV-SIG signal: gnss=%d sv=%d sig=%d freq=%d cno=%d "
                "quality=%d corr_source=%d flags=0x%04x",
                signal.gnss_id,
                signal.sv_id,
                signal.sig_id,
                signal.freq_id,
                signal.cno_dbhz,
                signal.quality_ind,
                signal.corr_source,
                signal.sig_flags,
            )


def read_mixed_frames(
    gps: UbloxGps, decoder: MixedF9PFrameDecoder, max_bytes: int
) -> List[StreamFrame]:
    """Read a bounded amount of SPI data without continuous empty polling."""
    frames: List[StreamFrame] = []
    for _ in range(max_bytes):
        data = gps.hard_port.read(1)
        if not data:
            break

        # Outside a UBX frame 0xFF is the SPI idle value.  Inside a validated
        # UBX frame it can be payload data, so preserve it for the decoder.
        if data == b"\xff" and not decoder.is_mid_ubx_frame:
            break
        frames.extend(decoder.feed(data))
    return frames


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--gga-only",
        action="store_true",
        help="suppress raw non-GGA NMEA sentences",
    )
    parser.add_argument(
        "--nav-sig-details",
        action="store_true",
        help="log each healthy carrier signal used by the navigation solution",
    )
    parser.add_argument(
        "--max-bytes-per-poll",
        type=int,
        default=64,
        help="maximum SPI bytes read per 1 ms poll (default: 64)",
    )
    args = parser.parse_args()
    if args.max_bytes_per_poll <= 0:
        parser.error("--max-bytes-per-poll must be positive")

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
    spi = spidev.SpiDev()
    gps = UbloxGps(spi)
    decoder = MixedF9PFrameDecoder()
    try:
        logging.info(
            "Reading F9P with the same SPI wrapper as gps_manager "
            "(bus 1, chip-select 0, 10 MHz, mode 0)."
        )
        logging.info("Stop gps_manager before this test so it does not consume the same stream.")
        logging.info("Mixed NMEA and UBX-NAV-SIG decoder enabled.")
        while True:
            frames = read_mixed_frames(gps, decoder, args.max_bytes_per_poll)
            if not frames:
                time.sleep(0.001)
                continue

            for frame in frames:
                if isinstance(frame, NmeaFrame):
                    nmea_callback(frame.sentence, raw_all=not args.gga_only)
                elif (
                    frame.message_class == UBX_NAV_CLASS
                    and frame.message_id == UBX_NAV_SIG_ID
                ):
                    try:
                        nav_sig_callback(parse_nav_sig(frame.payload), args.nav_sig_details)
                    except ValueError as error:
                        logging.warning("DROP invalid UBX-NAV-SIG: %s", error)
    except KeyboardInterrupt:
        logging.info("Stopped.")
    finally:
        spi.close()


if __name__ == "__main__":
    main()
