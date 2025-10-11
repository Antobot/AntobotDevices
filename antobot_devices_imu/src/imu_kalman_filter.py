#!/usr/bin/env python3
"""
IMU orientation estimation using Kalman filter.

This node implements a Kalman filter for estimating orientation from IMU data,
calculating roll, pitch and yaw angles. The implementation is based on the 
MPU6050 driver's Kalman filter algorithm, adapted for ROS 2.

Features:
- Three-axis independent Kalman filtering
- Configurable filter parameters
- Euler angles and quaternion output
- Adaptive bias estimation during stationary periods

Parameters:
- calibration_samples (int): Number of samples for initial bias calibration
- Q_angle (float): Process noise variance for angle
- Q_bias (float): Process noise variance for bias
- R_measure (float): Measurement noise variance
- gyro_still_thresh (float): Gyro threshold for stationary detection (rad/s)
- acc_still_thresh (float): Acc threshold for stationary detection (m/s²)
- bias_update_alpha (float): EMA coefficient for bias updates

Topics:
Subscribes to: /imu/data (sensor_msgs/Imu)
Publishes:
- /icm/orientation_euler_kalman (geometry_msgs/Vector3)
- /icm/orientation_quat_kalman (geometry_msgs/Quaternion)
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion


class KalmanFilter:
    """Kalman filter implementation for single axis."""
    def __init__(self, Q_angle: float, Q_bias: float, R_measure: float):
        self.Q_angle = Q_angle
        self.Q_bias = Q_bias
        self.R_measure = R_measure
        self.angle = 0.0
        self.bias = 0.0
        self.P = [[0.0, 0.0], [0.0, 0.0]]  # Error covariance matrix
    
    def update(self, new_angle: float, new_rate: float, dt: float) -> float:
        """Update Kalman filter with new measurement."""
        # Predict state
        rate = new_rate - self.bias
        self.angle += dt * rate
        
        # Predict error covariance
        self.P[0][0] += dt * (dt*self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt
        
        # Calculate Kalman gain
        S = self.P[0][0] + self.R_measure
        K = [self.P[0][0]/S, self.P[1][0]/S]
        
        # Update estimate
        y = new_angle - self.angle
        self.angle += K[0] * y
        self.bias += K[1] * y
        
        # Update error covariance
        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]
        
        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp
        
        return self.angle


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert Euler angles to quaternion."""
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


class IMUKalmanFilterNode(Node):
    def __init__(self):
        super().__init__('imu_kalman_filter_node')
        
        # Parameters
        self.declare_parameters(namespace='',
            parameters=[
                ('calibration_samples', 500),
                ('Q_angle', 0.001),
                ('Q_bias', 0.003),
                ('R_measure', 0.03),
                ('gyro_still_thresh', 0.02),
                ('acc_still_thresh', 0.3),
                ('bias_update_alpha', 0.001)
            ])
        
        # Kalman filters
        self.kalman_x = KalmanFilter(
            self.get_parameter('Q_angle').value,
            self.get_parameter('Q_bias').value,
            self.get_parameter('R_measure').value)
        
        self.kalman_y = KalmanFilter(
            self.get_parameter('Q_angle').value,
            self.get_parameter('Q_bias').value,
            self.get_parameter('R_measure').value)
        
        self.kalman_z = KalmanFilter(
            self.get_parameter('Q_angle').value,
            self.get_parameter('Q_bias').value,
            self.get_parameter('R_measure').value)
        
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
        self.euler_pub = self.create_publisher(Vector3, '/icm/orientation_euler_kalman', 10)
        self.quat_pub = self.create_publisher(Quaternion, '/icm/orientation_quat_kalman', 10)
        
        # Subscription
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)
        
        self.get_logger().info("IMU Kalman filter node started")

    def imu_callback(self, msg: Imu) -> None:
        # Timestamp and dt
        current_time = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        dt = 0.0 if self.prev_time is None else (current_time - self.prev_time)
        self.prev_time = current_time
        
        # Measurements
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        
        # Convert gyro from deg/s to rad/s
        gx = msg.angular_velocity.x * math.pi / 180.0
        gy = msg.angular_velocity.y * math.pi / 180.0
        gz = msg.angular_velocity.z * math.pi / 180.0
        
        # Initial calibration
        if not self.calibrated:
            self.bias_gx += gx
            self.bias_gy += gy
            self.bias_gz += gz
            self.calibration_count += 1
            
            if self.calibration_count >= self.get_parameter('calibration_samples').value:
                self.bias_gx /= self.calibration_count
                self.bias_gy /= self.calibration_count
                self.bias_gz /= self.calibration_count
                self.calibrated = True
                self.get_logger().info("IMU calibration completed")
            return
        
        # Stationary detection for adaptive bias update
        gyro_mag = abs(gx) + abs(gy) + abs(gz)
        acc_norm = math.sqrt(ax*ax + ay*ay + az*az)
        g = 9.80665
        
        if (gyro_mag < self.get_parameter('gyro_still_thresh').value and 
            abs(acc_norm - g) < self.get_parameter('acc_still_thresh').value):
            alpha = self.get_parameter('bias_update_alpha').value
            self.bias_gx = (1.0 - alpha) * self.bias_gx + alpha * gx
            self.bias_gy = (1.0 - alpha) * self.bias_gy + alpha * gy
            self.bias_gz = (1.0 - alpha) * self.bias_gz + alpha * gz
        
        # Corrected gyro readings
        gx_corr = gx - self.bias_gx
        gy_corr = gy - self.bias_gy
        gz_corr = gz - self.bias_gz
        
        # Calculate angles from accelerometer
        roll_acc = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.sqrt(ay*ay + az*az))
        
        # Kalman filter update
        if dt > 0.0:
            self.roll = self.kalman_x.update(roll_acc, gx_corr, dt)
            self.pitch = self.kalman_y.update(pitch_acc, gy_corr, dt)
            
            # Yaw integration (no magnetometer)
            self.yaw += gz_corr * dt
            self.yaw = (self.yaw + math.pi) % (2.0 * math.pi) - math.pi  # Wrap to [-π, π]
        
        # Publish orientation
        euler = Vector3()
        euler.x = self.roll
        euler.y = self.pitch
        euler.z = self.yaw
        self.euler_pub.publish(euler)
        self.quat_pub.publish(euler_to_quaternion(self.roll, self.pitch, self.yaw))


def main(args=None):
    rclpy.init(args=args)
    node = IMUKalmanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
