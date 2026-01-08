#!/usr/bin/env python3

# Copyright (c) 2025, ANTOBOT LTD.
# All rights reserved.
# ================================================================================

# Code Description:
#     This script provides an interactive configuration tool for the u-blox F9P
#     GNSS receiver using the UBX protocol. It builds and sends configuration
#     packets (CFG-VALSET, CFG-RST, MON-VER, CFG-RATE) over SPI or UART/USB, and
#     applies a unified configuration flow across different device roles.
#     - Support different operating roles via config_mode:
#         * Reset only
#         * Rover
#         * Local Base (RTCM output)
#         * Moving Rover (RELPOSNED only)
#         * Moving Base (RTCM + additional NMEA as needed)
#     - Configure measurement rate, serial baudrate
#     - Allow switching between RTCM MSM4 and MSM7 variants
#
# Contact: Qinwei Zhang, Huaide Wang
# email: qinwei.zhang@nicecart.ai

# ================================================================================

import spidev
import serial
import time
from typing import Tuple, List, Optional

#========================== OUTPUT BAUDRATE =====================
OUTPUT_BAUDRATE = 460800   # change here: 38400 / 460800

# Constants for UBX protocol header
UBX_HEADER_1 = 0xB5
UBX_HEADER_2 = 0x62

# Measurement rate raw values
MEAS_RATE_5HZ = 200   # 5 Hz  -> 200 ms
MEAS_RATE_8HZ = 125   # 8 Hz  -> 125 ms

# Role definitions
ROLE_RESET = 0
ROLE_ROVER = 1
ROLE_LOCAL_BASE = 2
ROLE_MOVING_ROVER = 3
ROLE_MOVING_BASE = 4

# Control interface selectors (hundreds digit)
IFACE_SPI = 1
IFACE_USB = 2
IFACE_UART = 3

# Output port selectors (tens digit)
OUT_NONE = 0
OUT_SPI = 1
OUT_UART1 = 2
OUT_UART2 = 3
OUT_USB = 4  # USB output

# Default RTCM MSM type (4 or 7)
DEFAULT_RTCM_MSM = 4


class UBXPacketBuilder:
    """Builds UBX configuration packets (CFG-VALSET, CFG-RST, MON-VER, CFG-RATE)."""

    def __init__(self):
        pass

    def set_length_fields(self, packet: bytearray, total_length: int) -> None:
        """Sets payload length fields of a UBX packet based on total packet size."""
        payload_length = total_length - 8
        packet[4] = payload_length & 0xFF
        packet[5] = (payload_length >> 8) & 0xFF

    def checksum(self, packet: bytearray) -> None:
        """Calculates UBX checksum over class, id, length and payload."""
        chk_a = 0
        chk_b = 0
        for b in packet[2:-2]:
            chk_a = (chk_a + b) & 0xFF
            chk_b = (chk_b + chk_a) & 0xFF
        packet[-2] = chk_a
        packet[-1] = chk_b

    def build_cfg_valset(self, length: int) -> bytearray:
        """Builds a CFG-VALSET packet header with RAM+FLASH layer selection."""
        packet = bytearray(length)
        packet[0] = UBX_HEADER_1
        packet[1] = UBX_HEADER_2
        packet[2] = 0x06
        packet[3] = 0x8A
        self.set_length_fields(packet, length)
        packet[6] = 0x00  # version
        packet[7] = 0x05  # layers: RAM + FLASH
        packet[8] = 0x00  # reserved
        packet[9] = 0x00  # reserved
        return packet

    def build_cfg_reset(self) -> bytearray:
        """Builds a CFG-RST packet to reset the F9P."""
        length = 21
        packet = bytearray(length)
        packet[0] = UBX_HEADER_1
        packet[1] = UBX_HEADER_2
        packet[2] = 0x06
        packet[3] = 0x09
        packet[4] = 0x0D
        packet[5] = 0x00
        packet[6:18] = bytes(
            [
                0xFF,
                0xFF,
                0xFF,
                0xFF,
                0x00,
                0x00,
                0x00,
                0x00,
                0xFF,
                0xFF,
                0xFF,
                0xFF,
            ]
        )
        packet[18] = 0x0F
        packet[19] = 0x00
        packet[20] = 0x00
        self.checksum(packet)
        return packet

    def build_mon_ver(self) -> bytearray:
        """Builds a MON-VER packet to request firmware version."""
        length = 8
        packet = bytearray(length)
        packet[0] = UBX_HEADER_1
        packet[1] = UBX_HEADER_2
        packet[2] = 0x0A
        packet[3] = 0x04
        packet[4] = 0x00
        packet[5] = 0x00
        packet[6] = 0x00
        packet[7] = 0x00
        self.checksum(packet)
        return packet

    def build_cfg_rate(self) -> bytearray:
        """Builds a CFG-RATE packet to read current measurement rate."""
        length = 8
        packet = bytearray(length)
        packet[0] = UBX_HEADER_1
        packet[1] = UBX_HEADER_2
        packet[2] = 0x06
        packet[3] = 0x08
        packet[4] = 0x00
        packet[5] = 0x00
        packet[6] = 0x00
        packet[7] = 0x00
        self.checksum(packet)
        return packet

    def build_cfg_rate_meas(self, period_ms: int) -> bytearray:
        """Builds CFG-VALSET packet to set CFG-RATE-MEAS (measurement period)."""
        length = 18
        packet = self.build_cfg_valset(length)
        packet[10] = 0x01
        packet[11] = 0x00
        packet[12] = 0x21
        packet[13] = 0x30
        packet[14] = period_ms & 0xFF
        packet[15] = (period_ms >> 8) & 0xFF
        self.checksum(packet)
        return packet

    def build_cfg_valset_keyvalue(self, key_id: Tuple[int, int, int, int], value: bytes) -> bytearray:
        """Builds CFG-VALSET packet to write a generic key-value pair."""
        length = 18 + len(value) - 2
        packet = self.build_cfg_valset(length)
        packet[10] = key_id[0]
        packet[11] = key_id[1]
        packet[12] = key_id[2]
        packet[13] = key_id[3]
        packet[14:14 + len(value)] = value
        self.checksum(packet)
        return packet


class F9PTransport:
    def __init__(self, port, use_spi: bool):
        self.port = port
        self.use_spi = use_spi

    def write(self, packet: bytearray):
        """Sends a UBX packet over SPI or UART."""
        if self.use_spi:
            self.port.writebytes(packet)
        else:
            self.port.write(packet)

    def read_feedback(self) -> bytearray:
        """Reads feedback bytes from F9P."""
        buffer_size = 2048
        if self.use_spi:
            received = bytearray(buffer_size)
            start = False
            index = 0
            it = 0
            while it < buffer_size:
                byte = self.port.readbytes(1)[0]
                if byte == UBX_HEADER_1:
                    start = True
                if start:
                    received[index] = byte
                    index += 1
                if start and index == buffer_size:
                    break
                it += 1
            return received
        else:
            return self.port.read(buffer_size)

    def write_and_read(self, packet: bytearray) -> bytearray:
        """Sends a packet and then reads feedback."""
        self.write(packet)
        return self.read_feedback()


def _key_int(value: int) -> Tuple[int, int, int, int]:
    """Converts a 32-bit UBX key integer into a little-endian 4-byte tuple."""
    return (
        value & 0xFF,
        (value >> 8) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 24) & 0xFF,
    )


# Base keys: NMEA / RELPOSNED / protocol enables (independent of MSM4/MSM7)
KEY_TABLE_BASE = {
    "GSV": {
        "SPI":   _key_int(0x209100C8),
        "UART1": _key_int(0x209100C5),
        "UART2": _key_int(0x209100C6),
        "USB":   _key_int(0x209100C7),
    },
    "RMC": {
        "SPI":   _key_int(0x209100AF),
        "UART1": _key_int(0x209100AC),
        "UART2": _key_int(0x209100AD),
        "USB":   _key_int(0x209100AE),
    },
    "GSA": {
        "SPI":   _key_int(0x209100C3),
        "UART1": _key_int(0x209100C0),
        "UART2": _key_int(0x209100C1),
        "USB":   _key_int(0x209100C2),
    },
    "GGA": {
        "SPI":   _key_int(0x209100BE),
        "UART1": _key_int(0x209100BB),
        "UART2": _key_int(0x209100BC),
        "USB":   _key_int(0x209100BD),
    },
    "VTG": {
        "SPI":   _key_int(0x209100B4),
        "UART1": _key_int(0x209100B1),
        "UART2": _key_int(0x209100B2),
        "USB":   _key_int(0x209100B3),
    },
    "GLL": {
        "SPI":   _key_int(0x209100CD),
        "UART1": _key_int(0x209100CA),
        "UART2": _key_int(0x209100CB),
        "USB":   _key_int(0x209100CC),
    },
    "GST": {
        "SPI":   _key_int(0x209100D7),
        "UART1": _key_int(0x209100D4),
        "UART2": _key_int(0x209100D5),
        "USB":   _key_int(0x209100D6),
    },
    "GNS": {
        "SPI":   _key_int(0x209100B9),
        "UART1": _key_int(0x209100B6),
        "UART2": _key_int(0x209100B7),
        "USB":   _key_int(0x209100B8),
    },
    "RELPOSNED": {
        "SPI":   _key_int(0x20910091),
        "UART1": _key_int(0x2091008E),
        "UART2": _key_int(0x2091008F),
        "USB":   _key_int(0x20910090),
    },
    "UBXENABLE": {
        "SPI":   _key_int(0x107A0001),
        "UART1": _key_int(0x10740001),
        "UART2": _key_int(0x10760001),
        "USB":   _key_int(0x10780001),
    },
    "NMEAENABLE": {
        "SPI":   _key_int(0x107A0002),
        "UART1": _key_int(0x10740002),
        "UART2": _key_int(0x10760002),
        "USB":   _key_int(0x10780002),
    },
    "RTCM3XENABLE": {
        "SPI":   _key_int(0x107A0004),
        "UART1": _key_int(0x10740004),
        "UART2": _key_int(0x10760004),
        "USB":   _key_int(0x10780004),
    },
}

# RTCM MSM4 keys
KEY_TABLE_RTCM4 = {
    "RTCM1074": {
        "SPI":   _key_int(0x20910362),
        "UART1": _key_int(0x2091035F),
        "UART2": _key_int(0x20910360),
        "USB":   _key_int(0x20910361),
    },
    "RTCM1084": {
        "SPI":   _key_int(0x20910367),
        "UART1": _key_int(0x20910364),
        "UART2": _key_int(0x20910365),
        "USB":   _key_int(0x20910366),
    },
    "RTCM1094": {
        "SPI":   _key_int(0x2091036C),
        "UART1": _key_int(0x20910369),
        "UART2": _key_int(0x2091036A),
        "USB":   _key_int(0x2091036B),
    },
    "RTCM1124": {
        "SPI":   _key_int(0x20910371),
        "UART1": _key_int(0x2091036E),
        "UART2": _key_int(0x2091036F),
        "USB":   _key_int(0x20910370),
    },
    "RTCM1230": {
        "SPI":   _key_int(0x20910307),
        "UART1": _key_int(0x20910304),
        "UART2": _key_int(0x20910305),
        "USB":   _key_int(0x20910306),
    },
    "RTCM4072": {
        "SPI":   _key_int(0x20910302),
        "UART1": _key_int(0x209102FF),
        "UART2": _key_int(0x20910300),
        "USB":   _key_int(0x20910301),
    },
    "RTCM40721": {
        "UART1": _key_int(0x20910382),
        "UART2": _key_int(0x20910383),
        "USB":   _key_int(0x20910384),
        "SPI":   _key_int(0x20910385),
    },
}

# RTCM MSM7 keys
KEY_TABLE_RTCM7 = {
    "RTCM1077": {
        "SPI":   _key_int(0x209102D0),
        "UART1": _key_int(0x209102CD),
        "UART2": _key_int(0x209102CE),
        "USB":   _key_int(0x209102CF),
    },
    "RTCM1087": {
        "SPI":   _key_int(0x209102D5),
        "UART1": _key_int(0x209102D2),
        "UART2": _key_int(0x209102D3),
        "USB":   _key_int(0x209102D4),
    },
    "RTCM1097": {
        "SPI":   _key_int(0x2091031C),
        "UART1": _key_int(0x20910319),
        "UART2": _key_int(0x2091031A),
        "USB":   _key_int(0x2091031B),
    },
    "RTCM1127": {
        "SPI":   _key_int(0x209102DA),
        "UART1": _key_int(0x209102D7),
        "UART2": _key_int(0x209102D8),
        "USB":   _key_int(0x209102D9),
    },
}

# Baudrate keys for UART1/UART2
KEY_TABLE_BAUD = {
    "UART1": _key_int(0x40520001),  # CFG-UART1-BAUD
    "UART2": _key_int(0x40530001),  # CFG-UART2-BAUD
}


class F9PMessageMap:

    def __init__(self, rtcm_msm_variant: int = DEFAULT_RTCM_MSM):
        if rtcm_msm_variant not in (4, 7):
            rtcm_msm_variant = DEFAULT_RTCM_MSM
        self.rtcm_msm_variant = rtcm_msm_variant

    def _normalize_name(self, msg: str) -> str:
        return msg.replace("_", "").upper()

    def _port_name_from_config(self, config_mode: str) -> Optional[str]:
        """Derives logical output port name (SPI/UART1/UART2/USB) from config_mode."""
        if len(config_mode) < 2:
            return None
        iface = int(config_mode[0])
        outport = int(config_mode[1])

        if outport == OUT_SPI:
            return "SPI"
        if outport == OUT_UART1:
            return "UART1"
        if outport == OUT_UART2:
            return "UART2"
        if outport == OUT_USB:
            return "USB"

        if outport == OUT_NONE:
            if iface == IFACE_SPI:
                return "SPI"
            if iface == IFACE_USB:
                return "USB"
            if iface == IFACE_UART:
                return "UART1"

        return None

    def get_key_id(self, msg: str, config_mode: str) -> Optional[Tuple[int, int, int, int]]:
        """Returns UBX key tuple for the given message on the selected output port."""
        name = self._normalize_name(msg)
        port = self._port_name_from_config(config_mode)
        if port is None:
            return None

        # Base table (NMEA / RELPOSNED / protocol enables)
        if name in KEY_TABLE_BASE:
            return KEY_TABLE_BASE[name].get(port)

        # RTCM MSM4 / MSM7
        if name.startswith("RTCM"):
            # MSM7 mapping
            if self.rtcm_msm_variant == 7 and name in KEY_TABLE_RTCM7:
                return KEY_TABLE_RTCM7[name].get(port)
            # MSM4 mapping or 1230/4072
            if name in KEY_TABLE_RTCM4:
                return KEY_TABLE_RTCM4[name].get(port)

        return None

    def get_rtcm_full_list(self) -> List[str]:
        """Full RTCM list for base (MSM4 or MSM7 + 1230/4072)."""
        if self.rtcm_msm_variant == 7:
            return [
                "RTCM1077",
                "RTCM1087",
                "RTCM1097",
                "RTCM1127",
                "RTCM1230",
                "RTCM4072",
                "RTCM4072_1",
            ]
        else:
            return [
                "RTCM1074",
                "RTCM1084",
                "RTCM1094",
                "RTCM1124",
                "RTCM1230",
                "RTCM4072",
                "RTCM4072_1",
            ]

    def get_default_rover_messages(self) -> List[str]:
        """Rover messages"""
        return ["GST", "VTG"]

    def get_moving_rover_messages(self) -> List[str]:
        """Messages for moving rover: RELPOSNED only."""
        return ["RELPOSNED"]

    def get_moving_base_messages(self) -> List[str]:
        """Messages for moving base: RTCM + RELPOSNED."""
        return self.get_rtcm_full_list() + ["GST", "VTG"]


class F9PConfigurator:

    def __init__(
        self,
        config_mode: str,
        desired_messages: List[str],
        meas_rate: int,
        transport: F9PTransport,
        packet_builder: UBXPacketBuilder,
        mapper: F9PMessageMap,
        do_reset: bool = True,
    ):
        self.config_mode = config_mode
        self.desired_messages = desired_messages
        self.meas_rate = meas_rate
        self.output_baudrate = OUTPUT_BAUDRATE
        self.transport = transport
        self.builder = packet_builder
        self.mapper = mapper
        self.do_reset = do_reset

    def parse_config_mode(self):
        iface = int(self.config_mode[0])
        outport = int(self.config_mode[1])
        role = int(self.config_mode[2])
        return iface, outport, role

    def action_reset(self):
        """Resets F9P"""
        packet = self.builder.build_cfg_reset()
        self.transport.write(packet)
        self.transport.read_feedback()

    def action_get_version(self):
        """Queries and prints firmware version using MON-VER."""
        packet = self.builder.build_mon_ver()
        self.transport.write(packet)
        received = self.transport.read_feedback()
        start_pattern = b"FWVER="
        idx = received.find(start_pattern)
        if idx != -1:
            end = received.find(b"\x00", idx)
            version = received[idx:end]
            print("Firmware:", version)

    def action_set_meas_rate(self):
        """Sets and reads back measurement rate"""
        period_ms = MEAS_RATE_5HZ if self.meas_rate == 5 else MEAS_RATE_8HZ
        packet = self.builder.build_cfg_rate_meas(period_ms)
        self.transport.write_and_read(packet)
        packet_read = self.builder.build_cfg_rate()
        self.transport.write_and_read(packet_read)

    def action_set_output_baudrate(self):
        """Sets output baudrate"""
        port_name = self.mapper._port_name_from_config(self.config_mode)
        if port_name not in ("UART1", "UART2"):
            return
        key = KEY_TABLE_BAUD.get(port_name)
        if key is None:
            return
        value = int(self.output_baudrate).to_bytes(4, byteorder="little", signed=False)
        packet = self.builder.build_cfg_valset_keyvalue(key, value)
        key_hex = f"0x{(key[3] << 24 | key[2] << 16 | key[1] << 8 | key[0]):08X}"
        print(
            f"[SEND KEY] {port_name + '_BAUD':<10} KEY={key_hex}  Set      Baud={self.output_baudrate}"
        )
        self.transport.write(packet)
        self.transport.read_feedback()

    def action_set_out_message(self, msg: str, enable: bool):
        """Enables or disables a single message (NMEA/UBX/RTCM)."""
        key = self.mapper.get_key_id(msg, self.config_mode)
        if key is None:
            return
        port = self.mapper._port_name_from_config(self.config_mode)
        value = bytes([0x01 if enable else 0x00, 0x00])
        packet = self.builder.build_cfg_valset_keyvalue(key, value)
        key_hex = f"0x{(key[3] << 24 | key[2] << 16 | key[1] << 8 | key[0]):08X}"
        print(
            f"[SEND KEY] {msg:<10} KEY={key_hex}  {'Enabled' if enable else 'Disabled':<8} Port={port}"
        )
        self.transport.write(packet)
        self.transport.read_feedback()

    def action_disable_nmea(self):
        """Disable ALL NMEA sentences (GGA, GSA, GSV, GLL, RMC, VTG, GST, GNS)."""
        nmea_list = ["GGA", "GSA", "GSV", "GLL", "RMC", "VTG", "GST", "GNS"]
        for msg in nmea_list:
            key = self.mapper.get_key_id(msg, self.config_mode)
            if key is None:
                continue
            port = self.mapper._port_name_from_config(self.config_mode)
            value = bytes([0x00, 0x00])
            packet = self.builder.build_cfg_valset_keyvalue(key, value)

            key_hex = f"0x{(key[3] << 24 | key[2] << 16 | key[1] << 8 | key[0]):08X}"
            print(f"[SEND KEY] {msg:<10} KEY={key_hex}  Disabled  Port={port}")

            self.transport.write(packet)
            self.transport.read_feedback()

    def action_apply_message_config(self):
        """Applies message configuration according to role and desired_messages."""
        iface, outport, role = self.parse_config_mode()

        if role == ROLE_RESET:
            return

        if outport == OUT_NONE:
            return

        final_list = list(self.desired_messages)

        # Rover without custom list -> GST + VTG (same as old behaviour)
        if role == ROLE_ROVER and not final_list:
            final_list = self.mapper.get_default_rover_messages()

        # Local base: add RTCM list
        if role == ROLE_LOCAL_BASE:
            final_list += self.mapper.get_rtcm_full_list()

        # Moving rover: f9p2-style -> RELPOSNED only
        if role == ROLE_MOVING_ROVER:
            final_list = self.mapper.get_moving_rover_messages()

        # Moving base: f9p1-style -> RTCM + RELPOSNED
        if role == ROLE_MOVING_BASE:
            final_list = self.mapper.get_moving_base_messages()

        # Disable NMEA first
        self.action_disable_nmea()

        # Enable selected messages
        for msg in final_list:
            self.action_set_out_message(msg, True)

    def run(self):
        """Full configuration flow, combining both original scripts' logic."""
        iface, outport, role = self.parse_config_mode()

        if role == ROLE_RESET:
            if self.do_reset:
                self.action_reset()
            return

        # Step1: reset
        if self.do_reset:
            self.action_reset()
            time.sleep(0.3)

        # Step2: get firmware version (from new scripts)
        self.action_get_version()
        time.sleep(0.2)

        # Step3: set frequency (5 Hz for moving rover/base, else keep user value)
        if role in [ROLE_MOVING_ROVER, ROLE_MOVING_BASE]:
            self.meas_rate = 5
        self.action_set_meas_rate()
        time.sleep(0.2)

        # Step3.5: set output baudrate on the selected UART port (if applicable)
        self.action_set_output_baudrate()
        time.sleep(0.2)

        # Step4: configure output message set (NMEA / RTCM / RELPOSNED etc.)
        self.action_apply_message_config()
        time.sleep(0.2)

        print(
            "Configuration completed for config_mode:",
            self.config_mode,
            "(reset sent)" if self.do_reset else "(no reset)",
        )


def interactive_select(prompt: str, options: dict) -> str:
    """Simple terminal menu helper."""
    print(prompt)
    for key, label in options.items():
        print(f"{key}) {label}")
    while True:
        selection = input("> ").strip()
        if selection in options:
            return selection
        print("Invalid selection, try again.")


def main():
    # Step 1: select interface
    iface = interactive_select(
        "Select interface:",
        {
            "1": "SPI",
            "2": "USB",
            "3": "UART",
        },
    )

    # Step 2: select output port
    outport = interactive_select(
        "Select output port:",
        {
            "0": "No output configuration",
            "1": "SPI output",
            "2": "UART1 output",
            "3": "UART2 output",
            "4": "USB output",
        },
    )

    # Step 3: select role
    role = interactive_select(
        "Select role:",
        {
            "0": "Reset only",
            "1": "Rover",
            "2": "Local Base (RTCM output)",
            "3": "Moving Rover (RELPOSNED only)",
            "4": "Moving Base (RTCM + GSV + VTG)",
        },
    )

    # Step 4: select RTCM MSM type if needed (local base / moving base)
    rtcm_msm = DEFAULT_RTCM_MSM
    if role in ["2", "4"]:
        rtcm_msm_sel = interactive_select(
            "Select RTCM MSM type for base output:",
            {
                "4": "MSM4 (1074/1084/1094/1124 + 1230/4072)",
                "7": "MSM7 (1077/1087/1097/1127 + 1230/4072)",
            },
        )
        rtcm_msm = int(rtcm_msm_sel)

    # Step 4.5: select reset behaviour
    reset_choice = interactive_select(
        "Reset behaviour:",
        {
            "0": "Reset device then configure (recommended)",
            "1": "Do NOT reset, only apply configuration",
        },
    )
    do_reset = reset_choice == "0"

    config_mode = iface + outport + role
    print(f"\nSelected config_mode: {config_mode}, RTCM MSM: {rtcm_msm}, do_reset={do_reset}")

    device_name = "/dev/ttyACM0"
    device_baudrate = 38400
    desired_messages: List[str] = []
    meas_rate = 5

    iface_int = int(iface)

    if iface_int == IFACE_SPI:
        spi = spidev.SpiDev()
        spi.open(1, 0)
        spi.max_speed_hz = 7800000
        spi.mode = 0
        port = spi
        use_spi = True
    else:
        uart = serial.Serial(port=device_name, baudrate=device_baudrate, timeout=1)
        port = uart
        use_spi = False

    packet_builder = UBXPacketBuilder()
    mapper = F9PMessageMap(rtcm_msm_variant=rtcm_msm)
    transport = F9PTransport(port, use_spi)

    configurator = F9PConfigurator(
        config_mode=config_mode,
        desired_messages=desired_messages,
        meas_rate=meas_rate,
        transport=transport,
        packet_builder=packet_builder,
        mapper=mapper,
        do_reset=do_reset,
    )

    configurator.run()


if __name__ == "__main__":
    main()
