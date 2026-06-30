#!/usr/bin/env python3

# Copyright (c) 2026, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# # # Code Description:  Configures the moving-base BASE F9P (the urcu receiver) to
#                        OUTPUT moving-base RTCM on its UART2 (4072.0, 1074, 1084, 1094,
#                        1124, 1230) so the ROVER can resolve the baseline (-> heading).
#
#                        The urcu F9P is configured over SPI (its UART2 does NOT accept
#                        UBX config commands - that's why configuring over /dev/ttyTHS0
#                        returns no-ack). This reuses the proven SPI config in f9p_config.py
#                        (the same path configure_f9p() uses for the urcu base).
#                        Settings are saved to RAM + Flash (persist).
#
# IMPORTANT: the SPI bus is shared with the running urcu GPS node. STOP it first so SPI
#            is free, e.g. stop gpsManager (or kill the urcu /rtk node), then:
#                python3 base_config.py
#            Run once. Companion to rover_config.py.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import os
import sys
import spidev

# Reuse the proven config class / RTCM keys from f9p_config.py (same folder)
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from f9p_config import F9P_config


def main():
    print("Configuring BASE (urcu) F9P RTCM output over SPI ...")
    spi = spidev.SpiDev()
    spi.open(2, 0)               # spi1 - same as f9p_config / sfeSpiWrapper
    spi.max_speed_hz = 7800000
    spi.mode = 0

    cfg = F9P_config(spi, [], 8, "spi")
    # Enables RTCM 4072.0/1074/1084/1094/1124/1230 on UART2 + sets UART2 baud 460800,
    # all over SPI (writebytes). Saved to RAM+Flash via the 0x05 layer in prepare_cfg_packet.
    cfg.config_uart2_rtcm()

    spi.close()
    print("Base RTCM output configured over SPI (RAM+Flash).")
    print("Re-run gps_movingbase.py; expect 'RTCMFramer: relayed ... head=0xd3' and diffSoln=True.")


if __name__ == "__main__":
    main()
