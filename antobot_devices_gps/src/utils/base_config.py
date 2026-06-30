#!/usr/bin/env python3

# Copyright (c) 2026, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# # # Code Description:  Configures the moving-base BASE F9P (the urcu receiver on
#                        /dev/ttyTHS0) to OUTPUT moving-base RTCM on its UART2 so the
#                        ROVER can resolve the baseline (-> heading). Companion to
#                        rover_config.py. Uses the same RTCM message keys as f9p_config.py
#                        (CFG-MSGOUT-RTCM_3X_* on UART2: 4072.0, 1074, 1084, 1094, 1124, 1230).
#                        Saved to RAM + Flash (persists across reboot).
#
# Run once, with the GPS stack / moving-base node / corrections STOPPED so ttyTHS0 is free:
#     rosnode kill /nRTK 2>/dev/null ; fuser -k /dev/ttyTHS0 ; python3 base_config.py
#
# Pair with rover_config.py (rover side: RTCM input + RELPOSNED output).

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import serial
import time

PORT = "/dev/ttyTHS0"
BAUD = 460800

# Configuration layer bitmask for CFG-VALSET: bit0 RAM, bit1 BBR, bit2 Flash.
# 0x05 = RAM + Flash (persist), matching f9p_config.py.
LAYERS = 0x05

# RTCM_3X message output on UART2 (key = 0x2091_03_<low>, same low bytes as f9p_config.py).
# Each is a 1-byte rate value (1 = output every nav epoch).
CFG_ITEMS = [
    ("CFG-MSGOUT-RTCM_3X_TYPE4072_0_UART2", 0x20910300, 1, 1),
    ("CFG-MSGOUT-RTCM_3X_TYPE1074_UART2",   0x20910360, 1, 1),
    ("CFG-MSGOUT-RTCM_3X_TYPE1084_UART2",   0x20910365, 1, 1),
    ("CFG-MSGOUT-RTCM_3X_TYPE1094_UART2",   0x2091036a, 1, 1),
    ("CFG-MSGOUT-RTCM_3X_TYPE1124_UART2",   0x2091036f, 1, 1),
    ("CFG-MSGOUT-RTCM_3X_TYPE1230_UART2",   0x20910305, 1, 1),
]


def build_valset(key, value, val_bytes):
    """Build a UBX-CFG-VALSET message for a single key/value (little-endian)."""
    payload = bytearray([0x00, LAYERS, 0x00, 0x00])     # version, layers, reserved x2
    payload += int(key).to_bytes(4, "little")           # key id
    payload += int(value).to_bytes(val_bytes, "little") # value

    msg = bytearray([0xB5, 0x62, 0x06, 0x8A])           # sync + CFG-VALSET (0x06 0x8A)
    msg += len(payload).to_bytes(2, "little")           # length
    msg += payload

    ck_a = ck_b = 0                                      # Fletcher checksum over class..payload
    for b in msg[2:]:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    msg += bytes([ck_a, ck_b])
    return bytes(msg)


def main():
    print("Configuring BASE F9P RTCM output on %s @ %d ..." % (PORT, BAUD))
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(0.2)

    for name, key, value, vb in CFG_ITEMS:
        pkt = build_valset(key, value, vb)
        ser.reset_input_buffer()
        ser.write(pkt)
        time.sleep(0.1)
        resp = ser.read(128)
        if b"\xb5\x62\x05\x01" in resp:
            status = "ACK"
        elif b"\xb5\x62\x05\x00" in resp:
            status = "NAK"
        else:
            status = "no-ack (UART2 baud != 460800? or port busy)"
        print("  %-40s -> %s" % (name, status))

    ser.close()
    print("Base RTCM output configured. Re-run gps_movingbase.py; expect 'RTCMFramer: relayed ... head=0xd3' and diffSoln=True.")


if __name__ == "__main__":
    main()
