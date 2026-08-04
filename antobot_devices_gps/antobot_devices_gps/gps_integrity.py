#!/usr/bin/env python3
"""Quality-gate F9P NavSatFix messages and publish conservative covariance."""

import copy
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix

from antobot_devices_msgs.msg import GpsIntegrity, GpsQual, GpsSignalHealth, RTCM

from .gps_covariance import GpsCovarianceModel, SignalHealthSample
from .gps_integrity_checker import (
    GpsFixSample,
    GpsIntegrityChecker,
    GpsQualitySample,
    RtcmSample,
)


def stamp_to_seconds(stamp) -> Optional[float]:
    """Convert a ROS time message to seconds, treating zero as unavailable."""
    seconds = float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0
    return seconds if seconds > 0.0 else None


class GpsIntegrityNode(Node):
    """Publish an integrity state and a quality-gated NavSatFix copy."""

    def __init__(self) -> None:
        super().__init__("gps_integrity")

        self.declare_parameter("input_gps_topic", "/antobot_gps")
        self.declare_parameter("input_quality_topic", "/antobot_gps/quality")
        self.declare_parameter("input_signal_health_topic", "/antobot_gps/signal_health")
        self.declare_parameter("input_rtcm_topic", "/antobot_gps/rtcm")
        self.declare_parameter("output_integrity_topic", "/antobot_gps/integrity")
        self.declare_parameter("output_gps_topic", "/antobot_gps/quality_checked")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("gpsfix_timeout_s", 1.0)
        self.declare_parameter("quality_timeout_s", 1.0)
        self.declare_parameter("max_timestamp_delta_s", 0.25)
        self.declare_parameter("min_navsat_status", 0)
        self.declare_parameter("min_gps_quality", 1)
        self.declare_parameter("min_num_sats", 4)
        self.declare_parameter("max_h_acc_m", 10.0)
        self.declare_parameter("max_hdop", 5.0)
        self.declare_parameter("require_timestamp_alignment", True)
        self.declare_parameter("rtcm_timeout_s", 5.0)
        self.declare_parameter("require_rtcm", True)
        self.declare_parameter("rtcm_good_age_s", 1.0)
        self.declare_parameter("rtcm_warning_age_s", 2.0)
        self.declare_parameter("rtcm_warning_covariance_multiplier", 4.0)
        self.declare_parameter("rtcm_critical_covariance_multiplier", 25.0)
        self.declare_parameter("signal_health_timeout_s", 2.0)
        self.declare_parameter("require_signal_health", False)
        self.declare_parameter("min_horizontal_stddev_m", 0.02)
        self.declare_parameter("vertical_stddev_multiplier", 4.0)
        self.declare_parameter("nominal_hdop", 1.0)
        self.declare_parameter("fixed_covariance_multiplier", 1.0)
        self.declare_parameter("float_covariance_multiplier", 25.0)
        self.declare_parameter("differential_covariance_multiplier", 100.0)
        self.declare_parameter("standalone_covariance_multiplier", 400.0)
        self.declare_parameter("cno_good_dbhz", 40.0)
        self.declare_parameter("cno_warning_dbhz", 35.0)
        self.declare_parameter("cno_critical_dbhz", 25.0)
        self.declare_parameter("cno_warning_covariance_multiplier", 4.0)
        self.declare_parameter("cno_critical_covariance_multiplier", 25.0)
        self.declare_parameter("unavailable_covariance_m2", 1_000_000.0)

        input_gps_topic = self._string_parameter("input_gps_topic")
        input_quality_topic = self._string_parameter("input_quality_topic")
        input_signal_health_topic = self._string_parameter("input_signal_health_topic")
        input_rtcm_topic = self._string_parameter("input_rtcm_topic")
        output_integrity_topic = self._string_parameter("output_integrity_topic")
        output_gps_topic = self._string_parameter("output_gps_topic")
        publish_rate_hz = self._double_parameter("publish_rate_hz")
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        self.checker = GpsIntegrityChecker(
            gpsfix_timeout_s=self._double_parameter("gpsfix_timeout_s"),
            quality_timeout_s=self._double_parameter("quality_timeout_s"),
            max_timestamp_delta_s=self._double_parameter("max_timestamp_delta_s"),
            min_navsat_status=self._integer_parameter("min_navsat_status"),
            min_gps_quality=self._integer_parameter("min_gps_quality"),
            min_num_sats=self._integer_parameter("min_num_sats"),
            max_h_acc_m=self._double_parameter("max_h_acc_m"),
            max_hdop=self._double_parameter("max_hdop"),
            require_timestamp_alignment=self._bool_parameter("require_timestamp_alignment"),
            rtcm_timeout_s=self._double_parameter("rtcm_timeout_s"),
            require_rtcm=self._bool_parameter("require_rtcm"),
        )
        self.signal_health_timeout_s = self._double_parameter("signal_health_timeout_s")
        if self.signal_health_timeout_s <= 0.0:
            raise ValueError("signal_health_timeout_s must be positive")
        self.covariance_model = GpsCovarianceModel(
            min_horizontal_stddev_m=self._double_parameter("min_horizontal_stddev_m"),
            vertical_stddev_multiplier=self._double_parameter("vertical_stddev_multiplier"),
            nominal_hdop=self._double_parameter("nominal_hdop"),
            fixed_covariance_multiplier=self._double_parameter("fixed_covariance_multiplier"),
            float_covariance_multiplier=self._double_parameter("float_covariance_multiplier"),
            differential_covariance_multiplier=self._double_parameter(
                "differential_covariance_multiplier"
            ),
            standalone_covariance_multiplier=self._double_parameter(
                "standalone_covariance_multiplier"
            ),
            cno_good_dbhz=self._double_parameter("cno_good_dbhz"),
            cno_warning_dbhz=self._double_parameter("cno_warning_dbhz"),
            cno_critical_dbhz=self._double_parameter("cno_critical_dbhz"),
            cno_warning_covariance_multiplier=self._double_parameter(
                "cno_warning_covariance_multiplier"
            ),
            cno_critical_covariance_multiplier=self._double_parameter(
                "cno_critical_covariance_multiplier"
            ),
            rtcm_good_age_s=self._double_parameter("rtcm_good_age_s"),
            rtcm_warning_age_s=self._double_parameter("rtcm_warning_age_s"),
            rtcm_critical_age_s=self._double_parameter("rtcm_timeout_s"),
            rtcm_warning_covariance_multiplier=self._double_parameter(
                "rtcm_warning_covariance_multiplier"
            ),
            rtcm_critical_covariance_multiplier=self._double_parameter(
                "rtcm_critical_covariance_multiplier"
            ),
            unavailable_covariance_m2=self._double_parameter("unavailable_covariance_m2"),
            require_signal_health=self._bool_parameter("require_signal_health"),
            require_rtcm=self._bool_parameter("require_rtcm"),
        )

        self.latest_gpsfix: Optional[GpsFixSample] = None
        self.latest_quality: Optional[GpsQualitySample] = None
        self.latest_navsatfix: Optional[NavSatFix] = None
        self.latest_rtcm: Optional[RtcmSample] = None
        self.latest_signal_health: Optional[SignalHealthSample] = None
        self.latest_signal_health_time: Optional[float] = None
        self.last_reported_reason: Optional[str] = None

        self.gps_sub = self.create_subscription(
            NavSatFix, input_gps_topic, self.gpsfix_callback, 10
        )
        self.quality_sub = self.create_subscription(
            GpsQual, input_quality_topic, self.quality_callback, 10
        )
        self.signal_health_sub = self.create_subscription(
            GpsSignalHealth,
            input_signal_health_topic,
            self.signal_health_callback,
            10,
        )
        self.rtcm_sub = self.create_subscription(RTCM, input_rtcm_topic, self.rtcm_callback, 10)
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.integrity_pub = self.create_publisher(
            GpsIntegrity, output_integrity_topic, status_qos
        )
        self.gps_pub = self.create_publisher(NavSatFix, output_gps_topic, 10)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_integrity)

        self.get_logger().info(
            "GPS covariance gate: fix=%s, quality=%s, signal=%s, rtcm=%s, output=%s"
            % (
                input_gps_topic,
                input_quality_topic,
                input_signal_health_topic,
                input_rtcm_topic,
                output_gps_topic,
            )
        )

    def gpsfix_callback(self, msg: NavSatFix) -> None:
        now = time.monotonic()
        self.latest_gpsfix = GpsFixSample(
            received_at=now,
            stamp_s=stamp_to_seconds(msg.header.stamp),
            latitude=float(msg.latitude),
            longitude=float(msg.longitude),
            status=int(msg.status.status),
        )
        self.latest_navsatfix = msg
        self.publish_reweighted_fix(msg, now)

    def quality_callback(self, msg: GpsQual) -> None:
        self.latest_quality = GpsQualitySample(
            received_at=time.monotonic(),
            stamp_s=stamp_to_seconds(msg.stamp),
            gps_quality=int(msg.gps_qual_val),
            horizontal_accuracy_m=float(msg.h_acc),
            satellites=int(msg.num_sats),
            hdop=float(msg.hor_dil),
        )

    def signal_health_callback(self, msg: GpsSignalHealth) -> None:
        self.latest_signal_health = SignalHealthSample(
            fresh=True,
            valid=bool(msg.valid),
            cno_median_dbhz=float(msg.cno_median_dbhz),
            cno_p10_dbhz=float(msg.cno_p10_dbhz),
        )
        self.latest_signal_health_time = time.monotonic()

    def rtcm_callback(self, msg: RTCM) -> None:
        """Record the arrival of a base-station RTCM message at this rover."""
        self.latest_rtcm = RtcmSample(received_at=time.monotonic())

    def publish_reweighted_fix(self, msg: NavSatFix, now: float) -> None:
        integrity = self.checker.evaluate(
            now, self.latest_gpsfix, self.latest_quality, self.latest_rtcm
        )
        covariance = self.calculate_covariance(msg, integrity, now)
        output = copy.deepcopy(msg)
        output.position_covariance = [0.0] * 9
        output.position_covariance[0] = covariance.covariance_xx
        output.position_covariance[4] = covariance.covariance_yy
        output.position_covariance[8] = covariance.covariance_zz
        output.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.gps_pub.publish(output)

    def calculate_covariance(self, msg: NavSatFix, integrity, now: float):
        quality = self.latest_quality
        signal_health = self.current_signal_health(now)
        return self.covariance_model.calculate(
            source_covariance=msg.position_covariance,
            horizontal_accuracy_m=quality.horizontal_accuracy_m if quality else float("nan"),
            hdop=quality.hdop if quality else float("nan"),
            gps_quality=quality.gps_quality if quality else 0,
            integrity_ok=integrity.integrity_ok,
            signal_health=signal_health,
            rtcm_age_s=integrity.rtcm_age_s if integrity.rtcm_received else None,
        )

    def current_signal_health(self, now: float) -> Optional[SignalHealthSample]:
        if self.latest_signal_health is None or self.latest_signal_health_time is None:
            return None
        return SignalHealthSample(
            fresh=now - self.latest_signal_health_time <= self.signal_health_timeout_s,
            valid=self.latest_signal_health.valid,
            cno_median_dbhz=self.latest_signal_health.cno_median_dbhz,
            cno_p10_dbhz=self.latest_signal_health.cno_p10_dbhz,
        )

    def publish_integrity(self) -> None:
        now = time.monotonic()
        result = self.checker.evaluate(
            now, self.latest_gpsfix, self.latest_quality, self.latest_rtcm
        )
        covariance = None
        if self.latest_navsatfix is not None:
            covariance = self.calculate_covariance(self.latest_navsatfix, result, now)
        msg = GpsIntegrity()
        msg.stamp = self.get_clock().now().to_msg()
        msg.gpsfix_received = result.gpsfix_received
        msg.quality_received = result.quality_received
        msg.rtcm_received = result.rtcm_received
        msg.gpsfix_fresh = result.gpsfix_fresh
        msg.quality_fresh = result.quality_fresh
        msg.rtcm_fresh = result.rtcm_fresh
        msg.fix_valid = result.fix_valid
        msg.quality_valid = result.quality_valid
        msg.timestamps_aligned = result.timestamps_aligned
        msg.integrity_ok = result.integrity_ok
        msg.gpsfix_age_s = result.gpsfix_age_s
        msg.quality_age_s = result.quality_age_s
        msg.rtcm_age_s = result.rtcm_age_s
        msg.timestamp_delta_s = result.timestamp_delta_s
        if covariance is not None:
            msg.covariance_scale = covariance.covariance_scale
            msg.horizontal_stddev_m = covariance.horizontal_stddev_m
            msg.vertical_stddev_m = covariance.vertical_stddev_m
        else:
            msg.covariance_scale = 0.0
            msg.horizontal_stddev_m = 0.0
            msg.vertical_stddev_m = 0.0
        msg.navsat_status = result.navsat_status
        msg.gps_quality = result.gps_quality
        msg.reason = (
            result.reason
            if covariance is None
            else "%s;%s" % (result.reason, covariance.reason)
        )
        self.integrity_pub.publish(msg)

        if msg.reason != self.last_reported_reason:
            log = self.get_logger().info if result.integrity_ok else self.get_logger().warn
            log("GPS integrity: %s" % msg.reason)
            self.last_reported_reason = msg.reason

    def _string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _double_parameter(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _integer_parameter(self, name: str) -> int:
        return self.get_parameter(name).get_parameter_value().integer_value

    def _bool_parameter(self, name: str) -> bool:
        return self.get_parameter(name).get_parameter_value().bool_value


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpsIntegrityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
