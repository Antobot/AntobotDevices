#!/usr/bin/env python3
"""
Compare IMU orientation estimation methods from ROS2 bag data
- Euler angles from orientation quaternion
- Euler angles from accelerometer
- Euler angles from Kalman filter
"""

import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
import argparse
from datetime import datetime
from scipy.spatial.transform import Rotation as R

class IMUOrientationComparator:
    def __init__(self, bag_dir, output_dir=None):
        self.bag_dir = bag_dir
        self.output_dir = output_dir or "output/imu_orientation_comparison"
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 数据存储
        self.timestamps = []
        self.linear_accel = []  # [ax, ay, az]
        self.angular_vel = []   # [gx, gy, gz]
        self.orientations = []  # [qx, qy, qz, qw]
        
        # 结果存储
        self.euler_from_orientation = []  # 从orientation计算的欧拉角
        self.euler_from_accel = []        # 从加速度计计算的欧拉角
        self.euler_from_kalman = []       # 卡尔曼滤波计算的欧拉角
        
        print(f"Bag directory: {bag_dir}")
        print(f"Output directory: {self.output_dir}")
    
    def read_bag_data(self):
        """读取ROS2 bag文件中的IMU数据"""
        try:
            from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
        except ImportError:
            print("错误: 需要安装rosbag2_py包")
            print("请运行: pip install rosbag2_py")
            return False
        
        try:
            storage_options = StorageOptions(uri=self.bag_dir, storage_id='sqlite3')
            converter_options = ConverterOptions('', '')
            
            reader = SequentialReader()
            reader.open(storage_options, converter_options)
            
            topic_types = reader.get_all_topics_and_types()
            imu_topics = [topic.name for topic in topic_types if 'imu' in topic.name.lower() and 'data' in topic.name.lower()]
            
            if not imu_topics:
                print("错误: 在bag文件中没有找到IMU话题")
                print("找到的话题:")
                for topic in topic_types:
                    print(f"  - {topic.name} ({topic.type})")
                return False
            
            imu_topic = imu_topics[0]
            print(f"使用IMU话题: {imu_topic}")
            
            imu_msg_type = None
            for topic in topic_types:
                if topic.name == imu_topic:
                    imu_msg_type = get_message(topic.type)
                    break
            
            if imu_msg_type is None:
                print(f"错误: 无法获取话题 {imu_topic} 的消息类型")
                return False
            
            count = 0
            while reader.has_next():
                (topic, data, t) = reader.read_next()
                if topic == imu_topic:
                    try:
                        msg = deserialize_message(data, imu_msg_type)
                        
                        timestamp = t / 1e9
                        
                        ax = msg.linear_acceleration.x
                        ay = msg.linear_acceleration.y
                        az = msg.linear_acceleration.z
                        
                        gx = msg.angular_velocity.x
                        gy = msg.angular_velocity.y
                        gz = msg.angular_velocity.z
                        
                        qx = msg.orientation.x
                        qy = msg.orientation.y
                        qz = msg.orientation.z
                        qw = msg.orientation.w
                        
                        self.timestamps.append(timestamp)
                        self.linear_accel.append([ax, ay, az])
                        self.angular_vel.append([gx, gy, gz])
                        self.orientations.append([qx, qy, qz, qw])
                        
                        count += 1
                        
                    except Exception as e:
                        print(f"警告: 无法反序列化消息: {e}")
                        continue
            
            print(f"成功读取 {count} 个IMU数据点")
            
            if count == 0:
                print("错误: 没有读取到任何IMU数据")
                return False
                
            self.timestamps = np.array(self.timestamps)
            self.linear_accel = np.array(self.linear_accel)
            self.angular_vel = np.array(self.angular_vel)
            self.orientations = np.array(self.orientations)
            
            return True
            
        except Exception as e:
            print(f"读取bag文件时出错: {e}")
            return False
    
    def quaternion_to_euler(self, qx, qy, qz, qw):
        """将四元数转换为欧拉角 (roll, pitch, yaw)"""
        # 使用scipy的Rotation类，更准确
        rotation = R.from_quat([qx, qy, qz, qw])
        euler = rotation.as_euler('xyz', degrees=False)
        return euler[0], euler[1], euler[2]  # roll, pitch, yaw
    
    def calculate_euler_from_orientation(self):
        """从orientation四元数计算欧拉角"""
        print("计算从orientation四元数得到的欧拉角...")
        euler_angles = []
        for i in range(len(self.orientations)):
            qx, qy, qz, qw = self.orientations[i]
            roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)
            euler_angles.append([roll, pitch, yaw])
        return np.array(euler_angles)
    
    def calculate_euler_from_accelerometer(self):
        """从加速度计数据计算欧拉角（只能计算roll和pitch）"""
        print("计算从加速度计得到的欧拉角...")
        euler_angles = []
        
        # 加速度计校准（计算零偏）
        calibration_samples = min(500, len(self.linear_accel))
        bias_ax = np.mean(self.linear_accel[:calibration_samples, 0])
        bias_ay = np.mean(self.linear_accel[:calibration_samples, 1])
        bias_az = np.mean(self.linear_accel[:calibration_samples, 2])
        
        print(f"加速度计零偏校准: ax={bias_ax:.3f}, ay={bias_ay:.3f}, az={bias_az:.3f}")
        
        for i in range(len(self.linear_accel)):
            ax, ay, az = self.linear_accel[i]
            
            # 应用零偏校正
            ax_corr = ax 
            ay_corr = ay 
            az_corr = az 
            
            # 从加速度计计算roll和pitch（与test.py相同的公式）
            roll_acc = np.arctan2(-ay_corr, np.sqrt(ax_corr**2 + az_corr**2))
            pitch_acc = np.arctan2(ax_corr, np.sqrt(ay_corr**2 + az_corr**2))
            
            # 加速度计无法计算yaw，设为0
            yaw_acc = 0.0
            
            euler_angles.append([roll_acc, pitch_acc, yaw_acc])
        
        return np.array(euler_angles)
    
    def calculate_kalman_filter_orientation(self):
        """使用卡尔曼滤波计算欧拉角"""
        print("计算卡尔曼滤波欧拉角...")
        
        # 导入卡尔曼滤波器
        try:
            from imu_kalman_filter import KalmanFilter
        except ImportError:
            print("错误: 无法导入KalmanFilter，请确保imu_kalman_filter.py在相同目录")
            return None
        
        # 初始化卡尔曼滤波器参数
        Q_angle = 0.001
        Q_bias = 0.003
        R_measure = 0.03
        
        kalman_x = KalmanFilter(Q_angle, Q_bias, R_measure)
        kalman_y = KalmanFilter(Q_angle, Q_bias, R_measure)
        kalman_z = KalmanFilter(Q_angle, Q_bias, R_measure)
        
        # 滤波器状态
        roll, pitch, yaw = 0.0, 0.0, 0.0
        bias_gx, bias_gy, bias_gz = 0.0, 0.0, 0.0  # 不使用初始校准
        
        kalman_orientations = []
        prev_time = self.timestamps[0] if len(self.timestamps) > 0 else 0
        
        for i in range(len(self.timestamps)):
            t = self.timestamps[i]
            ax, ay, az = self.linear_accel[i]
            gx, gy, gz = self.angular_vel[i]
            
            dt = t - prev_time if i > 0 else 0.01
            prev_time = t
            
            # 校正后的角速度（偏置为0）
            gx_corr = gx - bias_gx
            gy_corr = gy - bias_gy
            gz_corr = gz - bias_gz
            
            # 从加速度计计算角度（与test.py中的公式对齐）
            roll_acc = np.arctan2(-ay, np.sqrt(ax**2 + az**2))
            pitch_acc = np.arctan2(ax, np.sqrt(ay**2 + az**2))
            
            # 卡尔曼滤波更新
            if dt > 0:
                roll = kalman_x.update(roll_acc, gx_corr, dt)
                pitch = kalman_y.update(pitch_acc, gy_corr, dt)
                yaw += gz* dt
                
                # 角度归一化
                roll = self.normalize_angle(roll)
                pitch = self.normalize_angle(pitch)
            
            kalman_orientations.append([roll, pitch, yaw])
        
        return np.array(kalman_orientations)
    
    def normalize_angle(self, angle):
        """将角度归一化到[-π, π]范围内"""
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def calculate_all_orientations(self):
        """计算所有三种欧拉角"""
        self.euler_from_orientation = self.calculate_euler_from_orientation()
        self.euler_from_accel = self.calculate_euler_from_accelerometer()
        self.euler_from_kalman = self.calculate_kalman_filter_orientation()
        
        if self.euler_from_kalman is None:
            print("警告: 卡尔曼滤波计算失败，只比较前两种方法")
    
    def plot_comparison(self):
        """绘制三种欧拉角的对比图"""
        if len(self.timestamps) == 0:
            print("错误: 没有数据可绘制")
            return
        
        # 计算相对时间（从0开始）
        relative_time = self.timestamps - self.timestamps[0]
        
        # 创建图形
        fig, axes = plt.subplots(3, 1, figsize=(14, 10))
        
        angles = ['Roll', 'Pitch', 'Yaw']
        colors = {
            'orientation': '#1f77b4',  # 蓝色
            'acceleration': '#ff7f0e', # 橙色
            'kalman': '#2ca02c'        # 绿色
        }
        
        for i in range(3):
            # 绘制从orientation计算的欧拉角
            axes[i].plot(relative_time, np.degrees(self.euler_from_orientation[:, i]), 
                        label='From Orientation', 
                        color=colors['orientation'], 
                        linewidth=2.0)
            
            # 绘制从加速度计计算的欧拉角（yaw为0，不绘制）
            if i < 2:  # 只绘制roll和pitch
                axes[i].plot(relative_time, np.degrees(self.euler_from_accel[:, i]), 
                            label='From Accelerometer', 
                            color=colors['acceleration'], 
                            linewidth=1.5)
            
            # 绘制卡尔曼滤波计算的欧拉角
            if self.euler_from_kalman is not None:
                axes[i].plot(relative_time, np.degrees(self.euler_from_kalman[:, i]), 
                            label='Kalman Filter', 
                            color=colors['kalman'], 
                            linewidth=1.5)
            
            axes[i].set_ylabel(f'{angles[i]} (degrees)')
            axes[i].legend(loc='upper right')
            axes[i].grid(True, alpha=0.3)
        
        axes[2].set_xlabel('Time (s)')
        axes[0].set_title('IMU Orientation Estimation Methods Comparison')
        
        plt.tight_layout()
        
        # 保存图像
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = os.path.join(self.output_dir, f"orientation_comparison_{timestamp}.png")
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"对比图已保存到: {output_path}")
        
        plt.show()
        
        # 绘制误差图（相对于orientation的误差）
        self.plot_errors(relative_time)
    
    def plot_errors(self, relative_time):
        """绘制相对于orientation的误差图"""
        if self.euler_from_kalman is None:
            print("无法绘制误差图：卡尔曼滤波数据不可用")
            return
        
        # 计算误差
        acc_errors = self.euler_from_accel - self.euler_from_orientation
        kalman_errors = self.euler_from_kalman - self.euler_from_orientation
        
        # 角度误差需要特殊处理（周期性）
        acc_roll_errors = np.arctan2(np.sin(acc_errors[:, 0]), np.cos(acc_errors[:, 0]))
        acc_pitch_errors = np.arctan2(np.sin(acc_errors[:, 1]), np.cos(acc_errors[:, 1]))
        
        kalman_roll_errors = np.arctan2(np.sin(kalman_errors[:, 0]), np.cos(kalman_errors[:, 0]))
        kalman_pitch_errors = np.arctan2(np.sin(kalman_errors[:, 1]), np.cos(kalman_errors[:, 1]))
        kalman_yaw_errors = kalman_errors[:, 2]  # yaw不进行周期性处理
        
        fig, axes = plt.subplots(3, 1, figsize=(14, 10))
        
        angles = ['Roll', 'Pitch', 'Yaw']
        acc_angle_errors = [acc_roll_errors, acc_pitch_errors, np.zeros_like(acc_roll_errors)]  # yaw误差为0
        kalman_angle_errors = [kalman_roll_errors, kalman_pitch_errors, kalman_yaw_errors]
        
        colors = {
            'acceleration': '#ff7f0e',  # 橙色
            'kalman': '#2ca02c'         # 绿色
        }
        
        for i in range(3):
            # 绘制加速度计误差（只绘制roll和pitch）
            if i < 2:
                axes[i].plot(relative_time, np.degrees(acc_angle_errors[i]), 
                            color=colors['acceleration'], 
                            linewidth=1.5, 
                            label='Accelerometer Error')
            
            # 绘制卡尔曼滤波误差
            axes[i].plot(relative_time, np.degrees(kalman_angle_errors[i]), 
                        color=colors['kalman'], 
                        linewidth=1.5, 
                        label='Kalman Filter Error')
            
            axes[i].set_ylabel(f'{angles[i]} Error (degrees)')
            axes[i].grid(True, alpha=0.3)
            axes[i].legend(loc='upper right')
        
        axes[2].set_xlabel('Time (s)')
        axes[0].set_title('Orientation Estimation Errors (Relative to Orientation Quaternion)')
        
        plt.tight_layout()
        
        # 保存图像
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = os.path.join(self.output_dir, f"orientation_errors_{timestamp}.png")
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"误差图已保存到: {output_path}")
        
        plt.show()
        
        # 计算并显示误差统计
        self.calculate_error_stats(acc_angle_errors, kalman_angle_errors)
    
    def calculate_error_stats(self, acc_errors, kalman_errors):
        """计算并显示误差统计"""
        print("\n误差统计 (相对于Orientation Quaternion, 度):")
        print("=" * 70)
        print(f"{'角度':<10} {'方法':<20} {'RMSE':<10} {'MAE':<10} {'Std':<10}")
        print("-" * 70)
        
        angles = ['Roll', 'Pitch', 'Yaw']
        
        for i, angle in enumerate(angles):
            if i < 2:  # 只计算roll和pitch的加速度计误差
                acc_rmse = np.degrees(np.sqrt(np.mean(acc_errors[i]**2)))
                acc_mae = np.degrees(np.mean(np.abs(acc_errors[i])))
                acc_std = np.degrees(np.std(acc_errors[i]))
            else:  # yaw的加速度计误差设为NaN
                acc_rmse = acc_mae = acc_std = float('nan')
            
            kalman_rmse = np.degrees(np.sqrt(np.mean(kalman_errors[i]**2)))
            kalman_mae = np.degrees(np.mean(np.abs(kalman_errors[i])))
            kalman_std = np.degrees(np.std(kalman_errors[i]))
            
            if i < 2:
                print(f"{angle:<10} {'Accelerometer':<20} {acc_rmse:<10.3f} {acc_mae:<10.3f} {acc_std:<10.3f}")
            print(f"{angle:<10} {'Kalman Filter':<20} {kalman_rmse:<10.3f} {kalman_mae:<10.3f} {kalman_std:<10.3f}")
            print("-" * 70)
    
    def save_comparison_data(self):
        """保存比较数据到CSV文件"""
        if len(self.timestamps) == 0:
            print("错误: 没有数据可保存")
            return
        
        # 创建数据数组
        data_to_save = np.column_stack((
            self.timestamps,
            np.degrees(self.euler_from_orientation[:, 0]),  # roll_orientation
            np.degrees(self.euler_from_orientation[:, 1]),  # pitch_orientation
            np.degrees(self.euler_from_orientation[:, 2]),  # yaw_orientation
            np.degrees(self.euler_from_accel[:, 0]),        # roll_accel
            np.degrees(self.euler_from_accel[:, 1]),        # pitch_accel
            np.degrees(self.euler_from_accel[:, 2]),        # yaw_accel (always 0)
        ))
        
        # 如果卡尔曼滤波数据可用，添加到数组中
        if self.euler_from_kalman is not None:
            kalman_data = np.column_stack((
                np.degrees(self.euler_from_kalman[:, 0]),   # roll_kalman
                np.degrees(self.euler_from_kalman[:, 1]),   # pitch_kalman
                np.degrees(self.euler_from_kalman[:, 2]),   # yaw_kalman
            ))
            data_to_save = np.column_stack((data_to_save, kalman_data))
        
        # 保存到CSV
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(self.output_dir, f"orientation_comparison_{timestamp}.csv")
        
        if self.euler_from_kalman is not None:
            header = "timestamp,roll_orientation_deg,pitch_orientation_deg,yaw_orientation_deg,roll_accel_deg,pitch_accel_deg,yaw_accel_deg,roll_kalman_deg,pitch_kalman_deg,yaw_kalman_deg"
        else:
            header = "timestamp,roll_orientation_deg,pitch_orientation_deg,yaw_orientation_deg,roll_accel_deg,pitch_accel_deg,yaw_accel_deg"
        
        np.savetxt(csv_path, data_to_save, delimiter=',', header=header, comments='')
        print(f"比较数据已保存到: {csv_path}")

def main():
    parser = argparse.ArgumentParser(description='Compare IMU orientation estimation methods from ROS2 bag')
    parser.add_argument('bag_dir', help='Path to the ROS2 bag directory')
    parser.add_argument('-o', '--output', help='Output directory for plots', default=None)
    
    args = parser.parse_args()
    
    if not os.path.exists(args.bag_dir):
        print(f"错误: bag目录不存在: {args.bag_dir}")
        return
    
    # 创建比较器
    comparator = IMUOrientationComparator(args.bag_dir, args.output)
    
    # 读取数据
    print("正在读取bag数据...")
    if not comparator.read_bag_data():
        print("读取bag数据失败")
        return
    
    # 计算所有欧拉角
    print("正在计算各种欧拉角...")
    comparator.calculate_all_orientations()
    
    # 绘制比较图
    print("正在绘制比较图...")
    comparator.plot_comparison()
    
    # 保存数据
    print("正在保存比较数据...")
    comparator.save_comparison_data()
    
    print("处理完成!")

if __name__ == "__main__":
    main()
