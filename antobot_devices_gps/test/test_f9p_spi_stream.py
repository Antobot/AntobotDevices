import struct

from antobot_devices_gps.f9p_spi_stream import (
    F9PSpiFrameReader,
    NmeaFrame,
    UBX_NAV_CLASS,
    UBX_NAV_SIG_ID,
    UbxFrame,
    _ubx_checksum,
    nav_sig_health,
    parse_nav_sig,
)


class FakeSpiRead:
    def __init__(self, data):
        self.data = data
        self.offset = 0
        self.requests = []

    def __call__(self, size):
        self.requests.append(size)
        result = self.data[self.offset : self.offset + size]
        self.offset += len(result)
        return result


def nav_sig_payload(signal_count=75, carrier_used=True):
    payload = bytearray(struct.pack("<IBBH", 123456, 0, signal_count, 0))
    for index in range(signal_count):
        payload.extend(
            struct.pack(
                "<BBBBhBBBBHI",
                index % 5,
                index + 1,
                0,
                0,
                -1,
                40,
                5,
                0,
                0,
                (1 << 4) if carrier_used else 0,
                0,
            )
        )
    return bytes(payload)


def nav_sig_frame(signal_count=75):
    payload = nav_sig_payload(signal_count)
    body = bytes((UBX_NAV_CLASS, UBX_NAV_SIG_ID)) + struct.pack("<H", len(payload)) + payload
    return b"\xb5\x62" + body + _ubx_checksum(body)


def test_nav_sig_is_read_in_bounded_chunks_without_losing_following_nmea():
    first_nmea = b"$GNRMC,one*00\r\n"
    second_nmea = b"$GNGGA,two*00\r\n"
    transport = FakeSpiRead(first_nmea + nav_sig_frame() + second_nmea)
    reader = F9PSpiFrameReader(transport, ubx_chunk_size=32)

    first = reader.read_frame()
    nav_sig = reader.read_frame()
    second = reader.read_frame()

    assert first == NmeaFrame(first_nmea.decode("ascii"))
    assert isinstance(nav_sig, UbxFrame)
    assert nav_sig.message_class == UBX_NAV_CLASS
    assert nav_sig.message_id == UBX_NAV_SIG_ID
    assert second == NmeaFrame(second_nmea.decode("ascii"))
    assert max(transport.requests) == 32
    assert transport.requests.count(32) > 1

    health = nav_sig_health(parse_nav_sig(nav_sig.payload))
    assert health.valid
    assert health.satellites_in_view == 75
    assert health.cno_median_dbhz == 40.0
    assert isinstance(health.cno_median_dbhz, float)


def test_nav_sig_uses_healthy_tracking_signals_before_carrier_solution_exists():
    health = nav_sig_health(parse_nav_sig(nav_sig_payload(signal_count=2, carrier_used=False)))

    assert health.valid
    assert health.satellites_in_view == 2
    assert health.cno_median_dbhz == 40.0


def test_reader_retries_spi_idle_bytes_before_the_nmea_frame():
    transport = FakeSpiRead(b"\xff\xff\xff$GNGGA,two*00\r\n")

    frame = F9PSpiFrameReader(transport).read_frame()

    assert frame == NmeaFrame("$GNGGA,two*00\r\n")
    assert transport.requests[:3] == [1, 1, 1]


def test_partial_nmea_survives_an_idle_gap_between_reader_calls():
    transport = FakeSpiRead(b"$GNGGA,partial\xff\xff\xff\xff\xff\xff\xff\xff,two*00\r\n")
    reader = F9PSpiFrameReader(transport)

    assert reader.read_frame() is None
    assert reader.read_frame() == NmeaFrame("$GNGGA,partial,two*00\r\n")
