#!/usr/bin/env python3
"""
Orientation estimation with adaptive drift compensation.

In some deployments a fixed bias calibration does not fully cancel yaw drift —
measurements may still wander at several degrees per minute.  This node
implements a simple adaptive bias estimator that continues to track the
gyroscope's slow bias changes during operation when the IMU is detected to be
stationary.  The result is a further reduction of yaw drift compared to the
static calibration used in ``orientation_node_filtered.py``.

How it works:

* At startup, the node performs an initial calibration to estimate the
  gyroscope biases using ``calibration_samples`` readings (as before).
* During operation, if the sensor appears stationary (angular rates below
  ``gyro_still_thresh`` and acceleration magnitude within ``acc_still_thresh`` of
  gravity), the biases are slowly updated using an exponential moving average
  governed by ``bias_update_alpha``.  This allows the system to track slow
  drift in the gyroscope bias caused by temperature changes or other
  environmental effects, without corrupting the orientation during motion.
* Roll and pitch are estimated using a complementary filter, and yaw is
  integrated from the corrected z‑axis angular rate.  As with the other
  implementations, no magnetometer is used, so large or long‑duration yaw
  rotations will still accumulate some error, but it is substantially reduced.

Parameters exposed on the node:

* ``calibration_samples`` (int): number of samples used for initial bias
  calibration.  Defaults to 500.
* ``alpha`` (double): complementary filter coefficient for roll/pitch.  Defaults
  to 0.98.
* ``gyro_still_thresh`` (double): rad/s threshold below which the sensor is
  considered stationary for bias updates.  Defaults to 0.02 rad/s.
* ``acc_still_thresh`` (double): m/s² threshold for the difference between
  measured acceleration magnitude and gravity.  Defaults to 0.3.
* ``bias_update_alpha`` (double): EWMA coefficient for adaptive bias updates.
  Defaults to 0.001.

The node publishes Euler angles on ``/icm/orientation_euler_compensated`` and
quaternions on ``/icm/orientation_quat_compensated``.
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    quat = Quaternion()
    quat.w = cr * cp * cy + sr * sp * sy
    quat.x = sr * cp * cy - cr * sp * sy
    quat.y = cr * sp * cy + sr * cp * sy
    quat.z = cr * cp * sy - sr * sp * cy
    return quat


class OrientationDriftCompensatedNode(Node):
    def __init__(self) -> None:
        super().__init__('orientation_drift_compensated_node')
        # Parameters
        self.declare_parameter('calibration_samples', 500)
        self.declare_parameter('alpha', 0.98)
        self.declare_parameter('gyro_still_thresh', 0.02)
        self.declare_parameter('acc_still_thresh', 0.3)
        self.declare_parameter('bias_update_alpha', 0.001)
        self.calibration_samples: int = self.get_parameter('calibration_samples').get_parameter_value().integer_value
        self.alpha: float = self.get_parameter('alpha').get_parameter_value().double_value
        self.gyro_still_thresh: float = self.get_parameter('gyro_still_thresh').get_parameter_value().double_value
        self.acc_still_thresh: float = self.get_parameter('acc_still_thresh').get_parameter_value().double_value
        self.bias_update_alpha: float = self.get_parameter('bias_update_alpha').get_parameter_value().double_value

        # Bias state
        self.bias_gx = 0.0
        self.bias_gy = 0.0
        self.bias_gz = 0.0
        self.calibration_count = 0
        self.calibrated = False

        # Orientation state
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.prev_time: Optional[float] = None

        # Publishers
        self.euler_pub = self.create_publisher(Vector3, '/icm/orientation_euler_compensated', 10)
        self.quat_pub = self.create_publisher(Quaternion, '/icm/orientation_quat_compensated', 10)
        # Subscription
        self.subscription = self.create_subscription(
            Imu,
            '/icm/data',
            self.imu_callback,
            10
        )

        self.get_logger().info(
            f"Drift compensated orientation node started with calibration_samples={self.calibration_samples},"
            f" alpha={self.alpha}, gyro_still_thresh={self.gyro_still_thresh},"
            f" acc_still_thresh={self.acc_still_thresh}, bias_update_alpha={self.bias_update_alpha}"
        )

    def imu_callback(self, msg: Imu) -> None:
        # Timestamp and dt
        current_time = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        dt = 0.0 if self.prev_time is None else (current_time - self.prev_time)
        self.prev_time = current_time

        # Measurements
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        gx_deg, gy_deg, gz_deg = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z

        # Convert to rad/s
        gx = gx_deg * math.pi / 180.0
        gy = gy_deg * math.pi / 180.0
        gz = gz_deg * math.pi / 180.0

        # Calibration phase
        if not self.calibrated:
            self.bias_gx += gx
            self.bias_gy += gy
            self.bias_gz += gz
            self.calibration_count += 1
            if self.calibration_count >= self.calibration_samples:
                self.bias_gx /= self.calibration_samples
                self.bias_gy /= self.calibration_samples
                self.bias_gz /= self.calibration_samples
                self.calibrated = True
                self.get_logger().info(
                    f"Initial bias calibration complete: bias_gx={self.bias_gx:.5f}, bias_gy={self.bias_gy:.5f}, bias_gz={self.bias_gz:.5f} (rad/s)"
                )
            return

        # Stationary detection for adaptive bias update
        # Compute angular velocity magnitude and acceleration magnitude
        gyro_mag = abs(gx) + abs(gy) + abs(gz)
        acc_norm = math.sqrt(ax * ax + ay * ay + az * az)
        g = 9.80665
        if gyro_mag < self.gyro_still_thresh and abs(acc_norm - g) < self.acc_still_thresh:
            # Stationary: update biases with exponential moving average
            self.bias_gx = (1.0 - self.bias_update_alpha) * self.bias_gx + self.bias_update_alpha * gx
            self.bias_gy = (1.0 - self.bias_update_alpha) * self.bias_gy + self.bias_update_alpha * gy
            self.bias_gz = (1.0 - self.bias_update_alpha) * self.bias_gz + self.bias_update_alpha * gz

        # Bias corrected gyro
        gx_corr = gx - self.bias_gx
        gy_corr = gy - self.bias_gy
        gz_corr = gz - self.bias_gz

        # Complementary filter integration
        if dt > 0.0:
            # Gyro integration for roll/pitch
            roll_gyro = self.roll + gx_corr * dt
            pitch_gyro = self.pitch + gy_corr * dt
            # Accelerometer angles
            roll_acc = math.atan2(ay, az)
            pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az))
            # Blend
            self.roll = self.alpha * roll_gyro + (1.0 - self.alpha) * roll_acc
            self.pitch = self.alpha * pitch_gyro + (1.0 - self.alpha) * pitch_acc
            # Yaw integration
            self.yaw += gz_corr * dt
            self.yaw = (self.yaw + math.pi) % (2.0 * math.pi) - math.pi

        # Publish orientation
        euler = Vector3()
        euler.x = self.roll
        euler.y = self.pitch
        euler.z = self.yaw
        quat = euler_to_quaternion(self.roll, self.pitch, self.yaw)
        self.euler_pub.publish(euler)
        self.quat_pub.publish(quat)

        # Debug logging (lower severity to avoid flooding logs)
        self.get_logger().info(
            f"Euler (deg): roll={math.degrees(self.roll):6.1f}, pitch={math.degrees(self.pitch):6.1f}, yaw={math.degrees(self.yaw):6.1f}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OrientationDriftCompensatedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
