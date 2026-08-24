#!/usr/bin/env python3

# Copyright (c) 2026, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# # # Code Description:  Configures the moving-base ROVER F9P (the second antenna,
#                        /dev/AntoF9P). Modelled on f9p_config.py but standalone.
#                        It sets, via UBX-CFG-VALSET (saved to RAM + Flash):
#                          - accept RTCM3 input  (so the rover applies the base's relayed 4072 corrections)
#                          - output UBX-NAV-RELPOSNED (relative position vector -> heading)
#                          - 8 Hz measurement rate
#                          - enable GPS / GLONASS / Galileo / BeiDou
#
# Run once with the rover plugged in:
#     python3 rover_config.py
#
# NOTE: this configures the ROVER only. The BASE (urcu F9P on /dev/ttyTHS0) must
#       separately be configured to OUTPUT moving-base RTCM (4072.0/4072.1 + MSM + 1230).

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import serial
import time

PORT = "/dev/AntoF9P"
BAUD = 460800

# Configuration layer bitmask for CFG-VALSET: bit0 RAM, bit1 BBR, bit2 Flash.
# 0x05 = RAM + Flash so the settings persist across reboots.
LAYERS = 0x05

# (name, key_id, value, value_size_bytes) - u-blox ZED-F9P configuration keys
CFG_ITEMS = [
    # --- accept RTCM3 input (rover must USE the relayed base corrections) ---
    ("CFG-USBINPROT-RTCM3X",                0x10770004, 1, 1),
    ("CFG-UART1INPROT-RTCM3X",              0x10730004, 1, 1),

    # --- output UBX-NAV-RELPOSNED (heading / relative position) ---
    ("CFG-MSGOUT-UBX_NAV_RELPOSNED_USB",    0x20910090, 1, 1),
    ("CFG-MSGOUT-UBX_NAV_RELPOSNED_UART1",  0x2091008e, 1, 1),

    # --- 8 Hz nav rate: CFG-RATE-MEAS = 125 ms (U2) ---
    ("CFG-RATE-MEAS",                       0x30210001, 125, 2),

    # --- enable constellations (so the rover can get its own fix: gnssFixOK) ---
    ("CFG-SIGNAL-GPS_ENA",                  0x1031001f, 1, 1),
    ("CFG-SIGNAL-GLO_ENA",                  0x10310025, 1, 1),
    ("CFG-SIGNAL-GAL_ENA",                  0x10310021, 1, 1),
    ("CFG-SIGNAL-BDS_ENA",                  0x10310022, 1, 1),
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
    print("Configuring ROVER F9P on %s @ %d ..." % (PORT, BAUD))
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
            status = "no-ack (check port/baud)"
        print("  %-40s -> %s" % (name, status))

    ser.close()
    print("Rover config sent (saved to RAM+Flash). Re-run gps_movingbase.py and watch gnssFixOK / diffSoln.")


if __name__ == "__main__":
    main()
