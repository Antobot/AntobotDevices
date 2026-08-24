#!/usr/bin/env python3

# Copyright (c) 2026, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# # # Code Description:  Configures the moving-base BASE F9P (the urcu receiver) over SPI
#                        to OUTPUT moving-base RTCM (4072.0, 1074, 1084, 1094, 1124, 1230)
#                        so the Jetson can read it on /dev/ttyTHS0 and relay it to the rover
#                        (Jetson-relayed moving base).
#
#                        The base is configured over SPI (its UARTs do NOT accept UBX config).
#                        RTCM output is enabled on BOTH UART1 and UART2 (ttyTHS0 is wired to
#                        UART1 in this setup; UART2 is harmless if unused), with the port
#                        output protocol set to RTCM3 and baud 460800 to match the Jetson.
#                        Saved to RAM + Flash (persists). Does NOT touch NMEA (GGA/GSV/...).
#
# IMPORTANT: the SPI bus is shared with the running urcu GPS node. STOP it first (free SPI),
#            then run once:
#                python3 base_config.py

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import time
import spidev

SPI_BUS, SPI_DEV = 2, 0      # spi1 - same as f9p_config / sfeSpiWrapper
SPI_SPEED = 7800000
LAYERS = 0x05                # RAM + Flash

# RTCM_3X message output keys (name, UART1 key, UART2 key); value is a 1-byte rate (1 = every epoch).
# UART2 keys match f9p_config.py; UART1 = UART2 - 1 (u-blox port order I2C,UART1,UART2,USB,SPI).
RTCM_MSGS = [
    ("RTCM_4072_0", 0x209102ff, 0x20910300),
    ("RTCM_1074",   0x2091035f, 0x20910360),
    ("RTCM_1084",   0x20910364, 0x20910365),
    ("RTCM_1094",   0x20910369, 0x2091036a),
    ("RTCM_1124",   0x2091036e, 0x2091036f),
    ("RTCM_1230",   0x20910304, 0x20910305),
]

# Port output protocol must allow RTCM3, and UART baud must match the Jetson (460800).
OTHER = [
    ("UART1OUTPROT-RTCM3X", 0x10740004, 1,      1),
    ("UART2OUTPROT-RTCM3X", 0x10760004, 1,      1),
    ("UART1-BAUDRATE",      0x40520001, 460800, 4),
    ("UART2-BAUDRATE",      0x40530001, 460800, 4),
]


def build_valset(key, value, val_bytes):
    """Build a UBX-CFG-VALSET message for a single key/value (little-endian)."""
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


def send(spi, name, key, value, vb):
    spi.writebytes(list(build_valset(key, value, vb)))
    time.sleep(0.05)
    print("  sent %-22s (key 0x%08x = %d)" % (name, key, value))


def main():
    print("Configuring BASE (urcu) F9P over SPI: RTCM out on UART1 + UART2 ...")
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEV)
    spi.max_speed_hz = SPI_SPEED
    spi.mode = 0

    for name, u1, u2 in RTCM_MSGS:
        send(spi, name + "_UART1", u1, 1, 1)
        send(spi, name + "_UART2", u2, 1, 1)
    for name, key, value, vb in OTHER:
        send(spi, name, key, value, vb)

    spi.close()
    print("Done (RAM+Flash). Base outputs RTCM on UART1 & UART2 @460800.")
    print("Note: SPI config is not ACK-checked here - verify via gps_movingbase.py:")
    print("      expect 'RTCMFramer: relayed ... head=0xd3' and diffSoln=True.")


if __name__ == "__main__":
    main()
