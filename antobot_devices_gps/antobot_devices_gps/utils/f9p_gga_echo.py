#!/usr/bin/env python3
"""Print raw F9P GGA sentences and their parsed fix fields from a ROS topic."""

import argparse

import pynmea2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


GGA_QUALITY_NAMES = {
    0: "invalid",
    1: "single",
    2: "dgps",
    4: "rtk_fixed",
    5: "rtk_float",
    6: "estimated",
}


class F9pGgaEcho(Node):
    """Subscribe to raw NMEA and print original plus decoded GGA information."""

    def __init__(self, topic: str, log_all_nmea: bool) -> None:
        super().__init__("f9p_gga_echo")
        self.log_all_nmea = log_all_nmea
        self.subscription = self.create_subscription(String, topic, self.nmea_callback, 50)
        self.get_logger().info("Listening for F9P NMEA on %s" % topic)

    def nmea_callback(self, msg: String) -> None:
        raw = msg.data.strip()
        if self.log_all_nmea:
            self.get_logger().info("NMEA raw: %s" % raw)

        try:
            sentence = pynmea2.parse(raw)
        except (pynmea2.ParseError, ValueError) as error:
            self.get_logger().warn("NMEA parse failed: %s; raw: %s" % (error, raw))
            return

        if sentence.sentence_type != "GGA":
            return

        self.get_logger().info("GGA raw: %s" % raw)
        try:
            quality = int(sentence.gps_qual)
            satellites = int(sentence.num_sats)
            hdop = float(sentence.horizontal_dil)
            latitude = float(sentence.latitude)
            longitude = float(sentence.longitude)
            altitude_m = float(sentence.altitude)
        except (TypeError, ValueError) as error:
            self.get_logger().warn("GGA fields invalid: %s; raw: %s" % (error, raw))
            return

        correction_age_s = sentence.age_gps_data or "unavailable"
        quality_name = GGA_QUALITY_NAMES.get(quality, "unknown")
        self.get_logger().info(
            "GGA fix: utc=%s lat=%.8f lon=%.8f alt=%.3f m quality=%d(%s) "
            "sats=%d hdop=%.2f correction_age=%s s"
            % (
                sentence.timestamp,
                latitude,
                longitude,
                altitude_m,
                quality,
                quality_name,
                satellites,
                hdop,
                correction_age_s,
            )
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/antobot_gps/nmea")
    parser.add_argument(
        "--log-all-nmea",
        action="store_true",
        help="Print every raw NMEA sentence before filtering to GGA.",
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = F9pGgaEcho(args.topic, args.log_all_nmea)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
