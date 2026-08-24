#!/usr/bin/env python3

# Copyright (c) 2026, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# # # Code Description:  MINIMAL config for the urcu F9P: enable NMEA GSA + GSV output on
#                        the SPI port + set NMEA protocol version to 4.11, so both carry the
#                        trailing "GNSS System ID" field. This feeds:
#                          - GSA -> gps_f9p._handle_gsa -> gpsQual.used_systems (1.2.0 PRN block)
#                          - GSV -> gps_f9p._handle_gsv -> /antobot_gps/constellation (1.2.1)
#
#                        Changes ONLY these three items - does NOT touch RTCM, message rates,
#                        UART settings, or moving-base. The existing working dual-GPS / RTCM
#                        config is left intact. Sent over SPI (the port gps_f9p reads the urcu
#                        on). Saved to RAM+Flash.
#
# Run with the whole software stack STOPPED (so the SPI bus is free), once:
#     python3 enable_gsa_gsv.py

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import time
import spidev

SPI_BUS, SPI_DEV = 2, 0      # spi1 - same as f9p_config / sfeSpiWrapper
SPI_SPEED = 7800000
LAYERS = 0x05                # RAM + Flash (persist)

# (name, key, value, value_bytes). value for MSGOUT = output every Nth nav epoch;
# 8 -> ~1 Hz (GSA/GSV change slowly and GSV is bulky, keeps SPI load minimal).
ITEMS = [
    # NMEA-GSA on SPI (satellites USED -> gpsQual.used_systems / 1.2.0 PRN block)
    ("CFG-MSGOUT-NMEA_ID_GSA_SPI", 0x209100c3, 8, 1),
    # NMEA-GSV on SPI (satellites IN VIEW -> /antobot_gps/constellation / 1.2.1 message)
    ("CFG-MSGOUT-NMEA_ID_GSV_SPI", 0x209100c8, 8, 1),
    # NMEA 4.11 so GSA/GSV include the trailing GNSS System ID. NMEA-format only:
    # does NOT affect RTCM, GGA, or moving-base.
    ("CFG-NMEA-PROTVER",           0x20930001, 42, 1),
]


def build_valset(key, value, val_bytes):
    payload = bytearray([0x00, LAYERS, 0x00, 0x00])
    payload += int(key).to_bytes(4, "little")
    payload += int(value).to_bytes(val_bytes, "little")
    msg = bytearray([0xB5, 0x62, 0x06, 0x8A])
    msg += len(payload).to_bytes(2, "little")
    msg += payload
    ck_a = ck_b = 0
    for b in msg[2:]:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    msg += bytes([ck_a, ck_b])
    return bytes(msg)


def main():
    print("Enabling NMEA GSA + GSV (SPI) + NMEA 4.11 on the urcu F9P over SPI ...")
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEV)
    spi.max_speed_hz = SPI_SPEED
    spi.mode = 0
    for name, key, value, vb in ITEMS:
        spi.writebytes(list(build_valset(key, value, vb)))
        time.sleep(0.05)
        print("  sent %-28s (0x%08x = %d)" % (name, key, value))
    spi.close()
    print("Done (RAM+Flash). Restart the gps node.")
    print("  used_systems (GSA) -> 1.2.0 PRN block; /antobot_gps/constellation (GSV) -> 1.2.1.")
    print("Nothing else changed - RTCM / rates / UART / moving-base untouched.")


if __name__ == "__main__":
    main()
