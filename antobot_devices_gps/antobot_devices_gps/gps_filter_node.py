#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from antobot_devices_msgs.msg import GpsQual


def haversine_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate great-circle distance in meters between two WGS84 points.
    """
    r = 6371000.0  # Earth radius in meters

    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return r * c


class GpsFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("gps_filter_node")

        # Topics
        self.declare_parameter("input_gps_topic", "/antobot_gps")
        self.declare_parameter("input_quality_topic", "/antobot_gps/quality")
        self.declare_parameter("output_gps_topic", "/antobot_gps/filtered_for_ekf")

        # Thresholds
        self.declare_parameter("max_jump_distance_m", 3.5)
        self.declare_parameter("max_low_speed_jump_distance_m", 3.0)
        self.declare_parameter("low_speed_threshold_mps", 0.5)
        self.declare_parameter("max_hacc_m", 2.5)
        self.declare_parameter("max_time_gap_s", 3.0)
        self.declare_parameter("min_valid_status", 1)  # 0=no fix, 1+=usable by your internal convention
        self.declare_parameter("log_rejections", True)

        input_gps_topic = self.get_parameter("input_gps_topic").get_parameter_value().string_value
        input_quality_topic = self.get_parameter("input_quality_topic").get_parameter_value().string_value
        output_gps_topic = self.get_parameter("output_gps_topic").get_parameter_value().string_value

        self.max_jump_distance_m = self.get_parameter("max_jump_distance_m").get_parameter_value().double_value
        self.max_low_speed_jump_distance_m = self.get_parameter("max_low_speed_jump_distance_m").get_parameter_value().double_value
        self.low_speed_threshold_mps = self.get_parameter("low_speed_threshold_mps").get_parameter_value().double_value
        self.max_hacc_m = self.get_parameter("max_hacc_m").get_parameter_value().double_value
        self.max_time_gap_s = self.get_parameter("max_time_gap_s").get_parameter_value().double_value
        self.min_valid_status = self.get_parameter("min_valid_status").get_parameter_value().integer_value
        self.log_rejections = self.get_parameter("log_rejections").get_parameter_value().bool_value

        self.gps_sub = self.create_subscription(NavSatFix, input_gps_topic, self.gps_callback, 10)
        self.qual_sub = self.create_subscription(GpsQual, input_quality_topic, self.quality_callback, 10)
        self.gps_pub = self.create_publisher(NavSatFix, output_gps_topic, 10)

        # Latest quality info
        self.latest_speed_mps: float = 0.0
        self.latest_hacc_m: float = 9999.0
        self.latest_status: int = 0
        self.latest_quality_time = self.get_clock().now()

        # Last accepted GPS point
        self.last_good_msg: Optional[NavSatFix] = None
        self.last_good_time = None

        self.accept_count = 0
        self.reject_count = 0

        self.get_logger().info(
            f"gps_filter_node started. Input: {input_gps_topic}, "
            f"Quality: {input_quality_topic}, Output: {output_gps_topic}"
        )

    def quality_callback(self, msg: GpsQual) -> None:
        try:
            # v_sog is km/h in your message definition
            self.latest_speed_mps = float(msg.v_sog) / 3.6
            self.latest_hacc_m = float(msg.h_acc)
            self.latest_status = int(msg.gps_qual_val)
            self.latest_quality_time = self.get_clock().now()
        except Exception as e:
            self.get_logger().error(f"quality_callback error: {e}")

    def gps_callback(self, msg: NavSatFix) -> None:
        try:
            if not self.is_basic_fix_valid(msg):
                self.reject(msg, "basic_fix_invalid")
                return

            if not self.is_quality_valid():
                self.reject(msg, "quality_invalid")
                return

            if self.last_good_msg is None:
                self.accept(msg, "first_fix")
                return

            dt = self.time_diff_sec(self.last_good_msg, msg)

            # If timestamp is weird or stale, fall back to current clock-based reset logic
            if dt is None or dt <= 0.0 or dt > self.max_time_gap_s:
                self.accept(msg, f"reset_by_time_gap dt={dt}")
                return

            dist_m = haversine_distance_m(
                self.last_good_msg.latitude,
                self.last_good_msg.longitude,
                msg.latitude,
                msg.longitude,
            )

            # Rule 1: general jump rejection
            if dist_m > self.max_jump_distance_m:
                self.reject(msg, f"jump_reject dist={dist_m:.2f}m dt={dt:.2f}s")
                return

            # Rule 2: low-speed but large move rejection
            if self.latest_speed_mps < self.low_speed_threshold_mps and dist_m > self.max_low_speed_jump_distance_m:
                self.reject(
                    msg,
                    f"low_speed_jump_reject dist={dist_m:.2f}m speed={self.latest_speed_mps:.2f}mps"
                )
                return

            self.accept(msg, f"accepted dist={dist_m:.2f}m dt={dt:.2f}s")

        except Exception as e:
            self.get_logger().error(f"gps_callback error: {e}")

    def is_basic_fix_valid(self, msg: NavSatFix) -> bool:
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return False
        if abs(msg.latitude) < 1e-9 and abs(msg.longitude) < 1e-9:
            return False
        if not (-90.0 <= msg.latitude <= 90.0):
            return False
        if not (-180.0 <= msg.longitude <= 180.0):
            return False
        return True

    def is_quality_valid(self) -> bool:
        # Rule 3: status check
        if self.latest_status < self.min_valid_status:
            return False

        # Rule 4: hAcc check
        if self.latest_hacc_m > self.max_hacc_m:
            return False

        return True

    def time_diff_sec(self, old_msg: NavSatFix, new_msg: NavSatFix) -> Optional[float]:
        try:
            t_old = old_msg.header.stamp.sec + old_msg.header.stamp.nanosec / 1e9
            t_new = new_msg.header.stamp.sec + new_msg.header.stamp.nanosec / 1e9
            return t_new - t_old
        except Exception:
            return None

    def accept(self, msg: NavSatFix, reason: str) -> None:
        self.last_good_msg = msg
        self.last_good_time = self.get_clock().now()
        self.gps_pub.publish(msg)
        self.accept_count += 1

        if self.accept_count % 50 == 1:
            self.get_logger().info(
                f"[GPS_FILTER_ACCEPT] total_accept={self.accept_count}, "
                f"total_reject={self.reject_count}, reason={reason}"
            )

    def reject(self, msg: NavSatFix, reason: str) -> None:
        self.reject_count += 1
        if self.log_rejections:
            self.get_logger().warn(
                f"[GPS_FILTER_REJECT] total_accept={self.accept_count}, "
                f"total_reject={self.reject_count}, lat={msg.latitude:.8f}, "
                f"lon={msg.longitude:.8f}, speed={self.latest_speed_mps:.2f}mps, "
                f"hacc={self.latest_hacc_m:.2f}, status={self.latest_status}, reason={reason}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpsFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
