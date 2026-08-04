#!/usr/bin/env python3
"""Aggregate NMEA GSV signal strength into a GPS signal-health topic."""

import time

import pynmea2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from antobot_devices_msgs.msg import GpsSignalHealth

from .gps_signal_health_core import GsvSignalAccumulator, SignalHealthSnapshot


class GpsSignalHealthNode(Node):
    """Publish C/N0 statistics using raw NMEA GSV sentences from the F9P node."""

    def __init__(self) -> None:
        super().__init__("gps_signal_health")
        self.declare_parameter("input_nmea_topic", "/antobot_gps/nmea")
        self.declare_parameter("output_signal_health_topic", "/antobot_gps/signal_health")
        self.declare_parameter("sample_timeout_s", 2.0)
        self.declare_parameter("weak_cno_dbhz", 25.0)

        input_topic = self._string_parameter("input_nmea_topic")
        output_topic = self._string_parameter("output_signal_health_topic")
        self.accumulator = GsvSignalAccumulator(
            self._double_parameter("sample_timeout_s"),
            self._double_parameter("weak_cno_dbhz"),
        )
        self.publisher = self.create_publisher(GpsSignalHealth, output_topic, 10)
        self.subscription = self.create_subscription(String, input_topic, self.nmea_callback, 50)
        self.get_logger().info(
            "GPS signal health: input=%s, output=%s" % (input_topic, output_topic)
        )

    def nmea_callback(self, msg: String) -> None:
        try:
            sentence = pynmea2.parse(msg.data)
        except (pynmea2.ParseError, ValueError):
            return

        if sentence.sentence_type != "GSV":
            return

        samples = []
        for index in range(1, 5):
            satellite_id = getattr(sentence, "sv_prn_num_%d" % index, None)
            cno_value = getattr(sentence, "snr_%d" % index, None)
            if satellite_id and cno_value:
                try:
                    samples.append((str(satellite_id), float(cno_value)))
                except ValueError:
                    continue

        now = time.monotonic()
        self.accumulator.add(now, sentence.talker, samples)
        snapshot = self.accumulator.snapshot(now)
        if snapshot is not None:
            self.publish(snapshot)

    def publish(self, snapshot: SignalHealthSnapshot) -> None:
        msg = GpsSignalHealth()
        msg.stamp = self.get_clock().now().to_msg()
        msg.valid = True
        msg.satellites_in_view = snapshot.satellites_in_view
        msg.constellation_count = snapshot.constellation_count
        msg.weak_signal_count = snapshot.weak_signal_count
        msg.cno_mean_dbhz = snapshot.cno_mean_dbhz
        msg.cno_median_dbhz = snapshot.cno_median_dbhz
        msg.cno_p10_dbhz = snapshot.cno_p10_dbhz
        self.publisher.publish(msg)

    def _string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _double_parameter(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpsSignalHealthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
