#!/usr/bin/env python3
"""Echo raw F9P serial data and decode GGA fixes for hardware debugging."""

import argparse
import logging
from typing import Dict, Optional

import pynmea2
import serial


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


def parse_gga(raw_line: str) -> Optional[Dict[str, object]]:
    """Parse one NMEA GGA sentence, returning None for all other input."""
    nmea_start = raw_line.find("$")
    if nmea_start < 0:
        return None

    try:
        sentence = pynmea2.parse(raw_line[nmea_start:].strip())
    except (pynmea2.ParseError, ValueError):
        return None

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


def nmea_callback(raw_line: str, raw_all: bool) -> None:
    """Log a raw serial line and its GGA fields when present."""
    gga = parse_gga(raw_line)
    if raw_all or gga is not None:
        logging.info("RAW: %s", raw_line.rstrip())
    if gga is None:
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


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0", help="F9P serial device")
    parser.add_argument("--baudrate", type=int, default=38400, help="serial baud rate")
    parser.add_argument(
        "--gga-only",
        action="store_true",
        help="suppress raw non-GGA lines",
    )
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
    try:
        port = serial.Serial(args.port, args.baudrate, timeout=1.0)
    except serial.SerialException as error:
        raise SystemExit("Cannot open %s: %s" % (args.port, error)) from error

    logging.info("Reading %s at %d baud. Press Ctrl-C to stop.", args.port, args.baudrate)
    try:
        while True:
            raw_bytes = port.readline()
            if not raw_bytes:
                continue
            raw_line = raw_bytes.decode("ascii", errors="replace")
            nmea_callback(raw_line, raw_all=not args.gga_only)
    except KeyboardInterrupt:
        logging.info("Stopped.")
    finally:
        port.close()


if __name__ == "__main__":
    main()
