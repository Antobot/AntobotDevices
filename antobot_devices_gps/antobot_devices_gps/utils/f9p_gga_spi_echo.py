#!/usr/bin/env python3
"""Echo raw F9P SPI NMEA data and decode GGA fixes for uRCU debugging."""

import argparse
import logging
import sys
import time
from pathlib import Path
from typing import Dict, Optional

import pynmea2
import spidev

# Allow running the script directly from the source tree.  A copied script
# requires the package to be available through `source install/setup.bash`.
PACKAGE_SOURCE_ROOT = Path(__file__).resolve().parents[2]
if str(PACKAGE_SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SOURCE_ROOT))

from antobot_devices_gps.ublox_gps.ublox_gps import UbloxGps
from antobot_devices_gps.f9p_spi_stream import (
    NmeaFrame,
    UBX_NAV_CLASS,
    UBX_NAV_SIG_ID,
    UbxFrame,
    nav_sig_health,
    parse_nav_sig,
)


QUALITY_NAMES = {
    0: "Invalid",
    1: "GPS",
    2: "DGPS",
    4: "RTK Fixed",
    5: "RTK Float",
    6: "Estimated",
}



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


def nav_sig_callback(report, show_details: bool) -> None:
    """Log C/N0, preferring carrier signals used by the navigation solution."""
    tracked = [
        signal
        for signal in report.signals
        if signal.quality_ind >= 5
        and not signal.unhealthy
        and signal.cno_dbhz > 0
    ]
    selected = [signal for signal in tracked if signal.carrier_used] or tracked
    carrier_used_count = sum(signal.carrier_used for signal in tracked)
    logging.info(
        "NAV-SIG: itow_ms=%d signals=%d cno_candidates=%d carrier_used=%d",
        report.itow_ms,
        len(report.signals),
        len(tracked),
        carrier_used_count,
    )
    if not selected:
        logging.warning("NAV-SIG has no healthy locked signal")
        return

    health = nav_sig_health(report)
    logging.info(
        "NAV-SIG C/N0: mean=%.1f median=%.1f p10=%.1f weak_lt25=%d unique_sv=%d",
        health.cno_mean_dbhz,
        health.cno_median_dbhz,
        health.cno_p10_dbhz,
        health.weak_signal_count,
        health.satellites_in_view,
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
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
    spi = spidev.SpiDev()
    gps = UbloxGps(spi)
    try:
        logging.info(
            "Reading F9P with the same SPI wrapper as gps_manager "
            "(bus 1, chip-select 0, 10 MHz, mode 0)."
        )
        logging.info("Stop gps_manager before this test so it does not consume the same stream.")
        logging.info("Mixed NMEA and UBX-NAV-SIG reader enabled (32-byte UBX chunks).")
        while True:
            frame = gps.stream_mixed_frame()
            if frame is None:
                time.sleep(0.001)
                continue

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
