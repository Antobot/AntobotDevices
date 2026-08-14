#!/usr/bin/env python3
"""Quality-gate F9P NavSatFix messages and publish conservative covariance."""

import copy
import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix

from antobot_devices_msgs.msg import GpsIntegrity, GpsQual, GpsSignalHealth, RTCM

from .gps_covariance import (
    COVARIANCE_PARAMETER_DEFAULTS,
    CovariancePolicy,
    GpsCovarianceModel,
    PersistentSignalFaultTracker,
    SignalHealthSample,
)
from .gps_integrity_checker import (
    FixedDurationTracker,
    GpsFixSample,
    GpsIntegrityChecker,
    GpsQualitySample,
    RtcmSample,
)


def stamp_to_seconds(stamp) -> Optional[float]:
    """Convert a ROS time message to seconds, treating zero as unavailable."""
    seconds = float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0
    return seconds if seconds > 0.0 else None


NODE_PARAMETER_DEFAULTS = {
    "input_gps_topic": "/antobot_gps",
    "input_quality_topic": "/antobot_gps/quality",
    "input_signal_health_topic": "/antobot_gps/signal_health",
    "input_rtcm_topic": "/antobot_gps/rtcm",
    "output_integrity_topic": "/antobot_gps/integrity",
    "output_gps_topic": "/antobot_gps/quality_checked",
    "publish_rate_hz": 10.0,
    "gpsfix_timeout_s": 1.0,
    "quality_timeout_s": 1.0,
    "max_timestamp_delta_s": 0.25,
    "min_navsat_status": 0,
    "min_gps_quality": 1,
    "min_num_sats": 4,
    "max_h_acc_m": 10.0,
    "require_timestamp_alignment": True,
    "signal_health_timeout_s": 2.0,
    "cno_severe_persistent_epochs": 3,
    "cno_satellite_drop_threshold": 3,
    **COVARIANCE_PARAMETER_DEFAULTS,
}


class GpsIntegrityNode(Node):
    """Publish an integrity state and a quality-gated NavSatFix copy."""

    def __init__(self) -> None:
        super().__init__("gps_integrity")

        self.declare_parameters("", list(NODE_PARAMETER_DEFAULTS.items()))
        parameters = {
            name: self.get_parameter(name).value for name in NODE_PARAMETER_DEFAULTS
        }
        input_gps_topic = parameters["input_gps_topic"]
        input_quality_topic = parameters["input_quality_topic"]
        input_signal_health_topic = parameters["input_signal_health_topic"]
        input_rtcm_topic = parameters["input_rtcm_topic"]
        output_integrity_topic = parameters["output_integrity_topic"]
        output_gps_topic = parameters["output_gps_topic"]
        publish_rate_hz = parameters["publish_rate_hz"]
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        self.checker = GpsIntegrityChecker(
            gpsfix_timeout_s=parameters["gpsfix_timeout_s"],
            quality_timeout_s=parameters["quality_timeout_s"],
            max_timestamp_delta_s=parameters["max_timestamp_delta_s"],
            min_navsat_status=parameters["min_navsat_status"],
            min_gps_quality=parameters["min_gps_quality"],
            min_num_sats=parameters["min_num_sats"],
            max_h_acc_m=parameters["max_h_acc_m"],
            max_hdop=parameters["max_hdop"],
            require_timestamp_alignment=parameters["require_timestamp_alignment"],
            rtcm_timeout_s=parameters["rtcm_timeout_s"],
            require_rtcm=parameters["require_rtcm"],
        )
        self.signal_health_timeout_s = parameters["signal_health_timeout_s"]
        if self.signal_health_timeout_s <= 0.0:
            raise ValueError("signal_health_timeout_s must be positive")
        policy_values = {
            name: parameters[name] for name in COVARIANCE_PARAMETER_DEFAULTS
        }
        self.covariance_model = GpsCovarianceModel(CovariancePolicy(**policy_values))

        self.latest_gpsfix: Optional[GpsFixSample] = None
        self.latest_quality: Optional[GpsQualitySample] = None
        self.latest_navsatfix: Optional[NavSatFix] = None
        self.latest_rtcm: Optional[RtcmSample] = None
        self.latest_signal_health: Optional[SignalHealthSample] = None
        self.latest_signal_health_time: Optional[float] = None
        # A receiver that never publishes NAV-SIG must eventually be treated
        # like any other persistent C/N0 outage.
        self.signal_health_unavailable_since: Optional[float] = time.monotonic()
        self.signal_fault_tracker = PersistentSignalFaultTracker(
            severe_cno_dbhz=parameters["cno_critical_dbhz"],
            min_consecutive_epochs=parameters["cno_severe_persistent_epochs"],
            min_satellite_drop=parameters["cno_satellite_drop_threshold"],
        )
        self.fixed_duration_tracker = FixedDurationTracker()
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
        self.rtcm_sub = self.create_subscription(
            RTCM, input_rtcm_topic, self.rtcm_callback, 10
        )
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
        now = time.monotonic()
        self.latest_signal_health = self.signal_fault_tracker.update(SignalHealthSample(
            fresh=True,
            valid=bool(msg.valid),
            cno_median_dbhz=float(msg.cno_median_dbhz),
            cno_p10_dbhz=float(msg.cno_p10_dbhz),
            satellites_in_view=int(msg.satellites_in_view),
        ))
        self.latest_signal_health_time = now
        if self._signal_health_usable(self.latest_signal_health):
            self.signal_health_unavailable_since = None
        elif self.signal_health_unavailable_since is None:
            self.signal_health_unavailable_since = now

    def rtcm_callback(self, msg: RTCM) -> None:
        """Record the arrival of a base-station RTCM message at this rover."""
        self.latest_rtcm = RtcmSample(received_at=time.monotonic())

    def publish_reweighted_fix(self, msg: NavSatFix, now: float) -> None:
        integrity = self.checker.evaluate(
            now, self.latest_gpsfix, self.latest_quality, self.latest_rtcm
        )
        fixed_duration_s = self.fixed_duration_tracker.update(
            now, integrity.integrity_ok and integrity.gps_quality == 4
        )
        covariance = self.calculate_covariance(msg, integrity, now, fixed_duration_s)
        output = copy.deepcopy(msg)
        output.position_covariance = [0.0] * 9
        output.position_covariance[0] = covariance.covariance_xx
        output.position_covariance[4] = covariance.covariance_yy
        output.position_covariance[8] = covariance.covariance_zz
        output.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.gps_pub.publish(output)

    def calculate_covariance(
        self, msg: NavSatFix, integrity, now: float, fixed_duration_s: float
    ):
        quality = self.latest_quality
        signal_health = self.current_signal_health(now)
        return self.covariance_model.calculate(
            source_covariance=msg.position_covariance,
            horizontal_accuracy_m=quality.horizontal_accuracy_m if quality else float("nan"),
            hdop=quality.hdop if quality else float("nan"),
            gps_quality=quality.gps_quality if quality else 0,
            fixed_duration_s=fixed_duration_s,
            integrity_ok=integrity.integrity_ok,
            signal_health=signal_health,
            rtcm_age_s=integrity.rtcm_age_s if integrity.rtcm_received else None,
        )

    def current_signal_health(self, now: float) -> Optional[SignalHealthSample]:
        if self.latest_signal_health is None or self.latest_signal_health_time is None:
            return SignalHealthSample(
                fresh=False,
                valid=False,
                cno_median_dbhz=float("nan"),
                cno_p10_dbhz=float("nan"),
                unavailable_duration_s=max(
                    0.0, now - self.signal_health_unavailable_since
                ),
            )
        fresh = now - self.latest_signal_health_time <= self.signal_health_timeout_s
        if fresh and self._signal_health_usable(self.latest_signal_health):
            unavailable_duration_s = 0.0
        else:
            if self.signal_health_unavailable_since is None:
                self.signal_health_unavailable_since = self.latest_signal_health_time
            unavailable_duration_s = max(0.0, now - self.signal_health_unavailable_since)
        return SignalHealthSample(
            fresh=fresh,
            valid=self.latest_signal_health.valid,
            cno_median_dbhz=self.latest_signal_health.cno_median_dbhz,
            cno_p10_dbhz=self.latest_signal_health.cno_p10_dbhz,
            satellites_in_view=self.latest_signal_health.satellites_in_view,
            severe_persistent_fault=self.latest_signal_health.severe_persistent_fault,
            unavailable_duration_s=unavailable_duration_s,
        )

    @staticmethod
    def _signal_health_usable(signal_health: SignalHealthSample) -> bool:
        return signal_health.valid and math.isfinite(signal_health.cno_median_dbhz)

    def publish_integrity(self) -> None:
        now = time.monotonic()
        result = self.checker.evaluate(
            now, self.latest_gpsfix, self.latest_quality, self.latest_rtcm
        )
        fixed_duration_s = self.fixed_duration_tracker.update(
            now, result.integrity_ok and result.gps_quality == 4
        )
        covariance = None
        if self.latest_navsatfix is not None:
            covariance = self.calculate_covariance(
                self.latest_navsatfix, result, now, fixed_duration_s
            )
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
        msg.fixed_duration_s = fixed_duration_s
        if covariance is not None:
            msg.covariance_scale = covariance.covariance_scale
            msg.horizontal_stddev_m = covariance.horizontal_stddev_m
        else:
            msg.covariance_scale = 0.0
            msg.horizontal_stddev_m = 0.0
        msg.navsat_status = result.navsat_status
        msg.gps_quality = result.gps_quality
        msg.reason = (
            result.reason
            if covariance is None
            else "%s;%s" % (result.reason, covariance.reason)
        )
        self.integrity_pub.publish(msg)

        if msg.reason != self.last_reported_reason:
            state = "valid" if result.integrity_ok else "invalid"
            # Keep one severity at this call site. rclpy rejects changing it
            # between state transitions when logging is throttled/cached.
            self.get_logger().info("GPS integrity (%s): %s" % (state, msg.reason))
            self.last_reported_reason = msg.reason

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
