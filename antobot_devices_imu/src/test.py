#!/usr/bin/env python3
"""
IMU Pure Integration Validation Script for NCLT Dataset
包含纯积分、改进纯积分和加速度计计算的姿态
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import os
import sys
import datetime

# 添加当前目录到路径，以便导入本地模块
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from read_imu import read_imu_basic, get_available_dates

class IMUPureIntegrationValidator:
    def __init__(self, dataset_date, start_ratio=0.0, end_ratio=1.0):
        self.dataset_date = dataset_date
        self.start_ratio = start_ratio
        self.end_ratio = end_ratio
        self.results = {}
        
        # 创建输出目录
        self.output_dir = "output/PureIntegration"
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 创建带时间戳的子目录，避免覆盖
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        ratio_suffix = f"_{int(start_ratio*100)}-{int(end_ratio*100)}"
        self.run_dir = os.path.join(self.output_dir, f"{dataset_date}_{timestamp}{ratio_suffix}")
        os.makedirs(self.run_dir, exist_ok=True)
        
        print(f"输出目录: {self.run_dir}")
        print(f"分析时间范围: {start_ratio*100:.0f}% - {end_ratio*100:.0f}%")
        
    def truncate_data(self, data, timestamps):
        """根据时间比例截断数据"""
        if len(timestamps) == 0:
            return data, timestamps
            
        total_duration = timestamps[-1] - timestamps[0]
        start_time = timestamps[0] + total_duration * self.start_ratio
        end_time = timestamps[0] + total_duration * self.end_ratio
        
        # 找到在时间范围内的索引
        start_idx = np.argmax(timestamps >= start_time)
        end_idx = np.argmax(timestamps >= end_time)
        
        if end_idx == 0:  # 如果end_time超过最大时间，使用最后一个索引
            end_idx = len(timestamps) - 1
            
        print(f"截断数据: 时间范围 [{start_time:.2f}, {end_time:.2f}] 秒")
        print(f"截断数据: 索引范围 [{start_idx}, {end_idx}], 共 {end_idx - start_idx + 1} 个数据点")
        
        return data[start_idx:end_idx+1], timestamps[start_idx:end_idx+1]

    def calculate_kalman_filter_orientation(self, imu_data):
        """使用卡尔曼滤波计算姿态"""
        print("计算卡尔曼滤波姿态...")
        
        # 从imu_kalman_filter导入KalmanFilter类
        from imu_kalman_filter import KalmanFilter
        
        # 初始化卡尔曼滤波器参数（与ROS2节点使用相同的参数）
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
        prev_time = imu_data[0, 0]
        
        for i in range(len(imu_data)):
            t, ax, ay, az, gx, gy, gz = imu_data[i]
            dt = t - prev_time if i > 0 else 0.01
            prev_time = t
            
            # 注释掉初始校准，直接使用0偏置
            # if not calibrated:
            #     # 校准代码...
            #     continue
            
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
                yaw += gz_corr * dt
                
                # 角度归一化
                roll = self.normalize_angle(roll)
                pitch = self.normalize_angle(pitch)
            
            kalman_orientations.append([t, roll, pitch, yaw])
        
        return np.array(kalman_orientations)
    
    def load_ground_truth(self):
        """加载地面真值数据 - 优先使用100Hz数据"""
        # 首先尝试加载100Hz数据
        gt_path_100hz = f"dataset/{self.dataset_date}/odometry_mu_100hz.csv"
        gt_path_normal = f"dataset/{self.dataset_date}/odometry_mu.csv"
        
        if os.path.exists(gt_path_100hz):
            gt_path = gt_path_100hz
            print(f"使用100Hz地面真值数据: {gt_path}")
        elif os.path.exists(gt_path_normal):
            gt_path = gt_path_normal
            print(f"使用常规地面真值数据: {gt_path}")
        else:
            print(f"警告: 地面真值文件不存在")
            print(f"  检查过: {gt_path_100hz}")
            print(f"  检查过: {gt_path_normal}")
            return None
            
        try:
            gt_data = np.loadtxt(gt_path, delimiter=",")
            self.gt_timestamps = gt_data[:, 0] / 1e6  # 转换为秒
            self.gt_positions = gt_data[:, 1:4]  # x, y, z
            self.gt_orientations = gt_data[:, 4:7]  # roll, pitch, yaw
            
            print(f"地面真值数据点数: {len(self.gt_timestamps)}")
            print(f"地面真值时间范围: {self.gt_timestamps[0]:.2f} - {self.gt_timestamps[-1]:.2f} 秒")
            
            # 计算地面真值数据的频率
            if len(self.gt_timestamps) > 1:
                gt_dt = np.diff(self.gt_timestamps)
                gt_freq = 1.0 / np.mean(gt_dt)
                print(f"地面真值数据频率: {gt_freq:.2f} Hz")
            
            return gt_data
        except Exception as e:
            print(f"加载地面真值数据时出错: {e}")
            return None
    
    def check_coordinate_system(self, imu_data):
        """检查坐标系一致性"""
        print("\n检查坐标系一致性...")
        
        # 检查加速度计数据
        print("加速度计数据统计:")
        print(f"  ax: 均值={np.mean(imu_data[:, 1]):.3f}, 标准差={np.std(imu_data[:, 1]):.3f}")
        print(f"  ay: 均值={np.mean(imu_data[:, 2]):.3f}, 标准差={np.std(imu_data[:, 2]):.3f}")
        print(f"  az: 均值={np.mean(imu_data[:, 3]):.3f}, 标准差={np.std(imu_data[:, 3]):.3f}")
        
        # 检查陀螺仪数据
        print("陀螺仪数据统计:")
        print(f"  gx: 均值={np.mean(imu_data[:, 4]):.3f}, 标准差={np.std(imu_data[:, 4]):.3f}")
        print(f"  gy: 均值={np.mean(imu_data[:, 5]):.3f}, 标准差={np.std(imu_data[:, 5]):.3f}")
        print(f"  gz: 均值={np.mean(imu_data[:, 6]):.3f}, 标准差={np.std(imu_data[:, 6]):.3f}")
        
        # 检查地面真值数据
        if hasattr(self, 'gt_orientations'):
            print("地面真值姿态统计:")
            print(f"  roll:  均值={np.mean(self.gt_orientations[:, 0]):.3f}, 范围=[{np.min(self.gt_orientations[:, 0]):.3f}, {np.max(self.gt_orientations[:, 0]):.3f}]")
            print(f"  pitch: 均值={np.mean(self.gt_orientations[:, 1]):.3f}, 范围=[{np.min(self.gt_orientations[:, 1]):.3f}, {np.max(self.gt_orientations[:, 1]):.3f}]")
            print(f"  yaw:   均值={np.mean(self.gt_orientations[:, 2]):.3f}, 范围=[{np.min(self.gt_orientations[:, 2]):.3f}, {np.max(self.gt_orientations[:, 2]):.3f}]")
    
    def normalize_angle(self, angle):
        """将角度归一化到[-π, π]范围内"""
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def unwrap_angle(self, angles):
        """解包角度，消除跳变"""
        unwrapped = np.zeros_like(angles)
        unwrapped[0] = angles[0]
        
        for i in range(1, len(angles)):
            diff = angles[i] - angles[i-1]
            if diff > np.pi:
                unwrapped[i] = unwrapped[i-1] + diff - 2 * np.pi
            elif diff < -np.pi:
                unwrapped[i] = unwrapped[i-1] + diff + 2 * np.pi
            else:
                unwrapped[i] = unwrapped[i-1] + diff
        
        return unwrapped
    
    def calculate_pure_integration_orientation(self, imu_data):
        """使用纯积分方法计算姿态（只使用陀螺仪数据）"""
        pure_integration_orientations = []
        
        # 零偏校准
        bias_gx, bias_gy, bias_gz = 0.0, 0.0, 0.0
        calibrated = False
        calibration_count = 0
        calibration_samples = 500
        
        # 初始姿态角
        roll, pitch, yaw = 0.0, 0.0, 0.0
        prev_time = imu_data[0, 0]
        
        for i in range(len(imu_data)):
            t, ax, ay, az, gx, gy, gz = imu_data[i]
            dt = t - prev_time if i > 0 else 0.01
            prev_time = t
            
            # 初始校准
            if not calibrated:
                bias_gx += gx
                bias_gy += gy
                bias_gz += gz
                calibration_count += 1
                
                if calibration_count >= calibration_samples:
                    bias_gx /= calibration_count
                    bias_gy /= calibration_count
                    bias_gz /= calibration_count
                    calibrated = True
                    print(f"纯积分IMU校准完成: bias_gx={bias_gx:.6f}, bias_gy={bias_gy:.6f}, bias_gz={bias_gz:.6f}")
                else:
                    continue
            
            # 校正后的角速度
            gx_corr = gx - bias_gx
            gy_corr = gy - bias_gy
            gz_corr = gz - bias_gz
            
            # 纯积分：只使用陀螺仪数据
            if dt > 0:
                # 直接对陀螺仪角速度进行积分
                roll += gx_corr * dt
                pitch += gy_corr * dt
                yaw += gz_corr * dt
                
                # 对roll和pitch进行归一化
                roll = self.normalize_angle(roll)
                pitch = self.normalize_angle(pitch)
                # yaw不进行归一化，避免跳变
            
            pure_integration_orientations.append([t, roll, pitch, yaw])
        
        return np.array(pure_integration_orientations)
    
    def calculate_improved_pure_integration(self, imu_data):
        """改进的纯积分方法，包含更好的零偏处理"""
        print("计算改进的纯积分姿态...")
        orientations = []
        
        # 使用更长的校准时间
        calibration_samples = 1000  # 增加到1000个样本
        bias_gx, bias_gy, bias_gz = 0.0, 0.0, 0.0
        calibrated = False
        
        roll, pitch, yaw = 0.0, 0.0, 0.0
        prev_time = imu_data[0, 0]
        
        for i in range(len(imu_data)):
            t, ax, ay, az, gx, gy, gz = imu_data[i]
            dt = t - prev_time if i > 0 else 0.01
            prev_time = t
            
            # 校准阶段
            if not calibrated:
                bias_gx += gx
                bias_gy += gy
                bias_gz += gz
                
                if i >= calibration_samples:
                    bias_gx /= calibration_samples
                    bias_gy /= calibration_samples
                    bias_gz /= calibration_samples
                    calibrated = True
                    print(f"改进校准完成: bias_gx={bias_gx:.6f}, bias_gy={bias_gy:.6f}, bias_gz={bias_gz:.6f}")
                else:
                    continue
            
            # 应用零偏校正
            gx_corr = gx - bias_gx
            gy_corr = gy - bias_gy
            gz_corr = gz - bias_gz
            
            # 简单的阈值滤波，去除微小噪声
            noise_threshold = 0.001  # rad/s
            if abs(gx_corr) < noise_threshold:
                gx_corr = 0.0
            if abs(gy_corr) < noise_threshold:
                gy_corr = 0.0
            if abs(gz_corr) < noise_threshold:
                gz_corr = 0.0
            
            # 纯积分
            roll += gx_corr * dt
            pitch += gy_corr * dt
            yaw += gz_corr * dt
            
            # 只对roll和pitch进行归一化
            roll = self.normalize_angle(roll)
            pitch = self.normalize_angle(pitch)
            
            orientations.append([t, roll, pitch, yaw])
        
        return np.array(orientations)
    
    def calculate_acceleration_orientation(self, imu_data):
        """使用加速度计计算roll和pitch角度"""
        print("计算加速度计姿态...")
        acc_orientations = []
        
        # 加速度计校准
        bias_ax, bias_ay, bias_az = 0.0, 0.0, 0.0
        calibrated = False
        calibration_count = 0
        calibration_samples = 500
        
        # 重力加速度大小 (m/s²)
        gravity = 9.81
        
        for i in range(len(imu_data)):
            t, ax, ay, az, gx, gy, gz = imu_data[i]
            
            # 初始校准
            if not calibrated:
                bias_ax += ax
                bias_ay += ay
                bias_az += az
                calibration_count += 1
                
                if calibration_count >= calibration_samples:
                    bias_ax /= calibration_count
                    bias_ay /= calibration_count
                    bias_az /= calibration_count
                    calibrated = True
                    print(f"加速度计校准完成: bias_ax={bias_ax:.6f}, bias_ay={bias_ay:.6f}, bias_az={bias_az:.6f}")
                else:
                    continue
            
            # 校正后的加速度
            # ax_corr = ax - bias_ax
            # ay_corr = ay - bias_ay
            # az_corr = az - bias_az
            
            ax_corr = ax 
            ay_corr = ay 
            az_corr = az 

            # 从加速度计计算roll和pitch
            # 注意：这里假设传感器坐标系为x前、y右、z下
            # roll_acc = np.arctan2(ay_corr, az_corr)
            # pitch_acc = np.arctan2(-ax_corr, np.sqrt(ay_corr**2 + az_corr**2))
            roll_acc = np.arctan2(-ay_corr, np.sqrt(ax_corr**2 + az_corr**2))
            pitch_acc = np.arctan2(ax_corr, np.sqrt(ay_corr**2 + az_corr**2))
            
            # 加速度计无法计算yaw，设为0
            yaw_acc = 0.0
            
            acc_orientations.append([t, roll_acc, pitch_acc, yaw_acc])
        
        return np.array(acc_orientations)
    
    def align_timestamps(self, imu_times, gt_times, imu_data, gt_data):
        """对齐时间戳"""
        aligned_imu = []
        aligned_gt = []
        aligned_gt_times = []
        
        # 确保我们只处理在IMU时间范围内的地面真值数据
        imu_start = imu_times[0]
        imu_end = imu_times[-1]
        
        # 找到在IMU时间范围内的地面真值数据点
        valid_gt_indices = np.where((gt_times >= imu_start) & (gt_times <= imu_end))[0]
        
        print(f"IMU时间范围: {imu_start:.2f} - {imu_end:.2f} 秒")
        print(f"地面真值时间范围: {gt_times[0]:.2f} - {gt_times[-1]:.2f} 秒")
        print(f"有效地面真值数据点数: {len(valid_gt_indices)}")
        
        for gt_idx in valid_gt_indices:
            gt_time = gt_times[gt_idx]
            gt_orient = gt_data[gt_idx]
            
            # 找到最接近的IMU时间戳
            time_diffs = np.abs(imu_times - gt_time)
            closest_idx = np.argmin(time_diffs)
            
            # 检查时间差是否在容忍范围内
            if time_diffs[closest_idx] < 0.05:  # 50ms容忍度
                aligned_imu.append(imu_data[closest_idx])
                aligned_gt.append(gt_orient)
                aligned_gt_times.append(gt_time)
        
        print(f"成功对齐的数据点数: {len(aligned_imu)}")
        
        return np.array(aligned_imu), np.array(aligned_gt), np.array(aligned_gt_times)
    
    def calculate_metrics(self, estimated, ground_truth):
        """计算性能指标"""
        errors = estimated - ground_truth
        
        # 角度误差需要特殊处理（周期性）
        roll_errors = np.arctan2(np.sin(errors[:, 0]), np.cos(errors[:, 0]))
        pitch_errors = np.arctan2(np.sin(errors[:, 1]), np.cos(errors[:, 1]))
        
        # 对于yaw，使用解包后的角度计算误差
        yaw_est_unwrapped = self.unwrap_angle(estimated[:, 2])
        yaw_gt_unwrapped = self.unwrap_angle(ground_truth[:, 2])
        yaw_errors = yaw_est_unwrapped - yaw_gt_unwrapped
        
        metrics = {
            'roll_rmse': np.sqrt(np.mean(roll_errors**2)),
            'pitch_rmse': np.sqrt(np.mean(pitch_errors**2)),
            'yaw_rmse': np.sqrt(np.mean(yaw_errors**2)),
            'roll_mae': np.mean(np.abs(roll_errors)),
            'pitch_mae': np.mean(np.abs(pitch_errors)),
            'yaw_mae': np.mean(np.abs(yaw_errors)),
            'roll_std': np.std(roll_errors),
            'pitch_std': np.std(pitch_errors),
            'yaw_std': np.std(yaw_errors)
        }
        
        return metrics
    
    def plot_comparison(self, pure_integration, acceleration_orientation, kalman_orientation, ground_truth, times):
        """绘制对比图 - 三种方法 vs 地面真值"""
        plt.ion()
        
        fig, axes = plt.subplots(3, 1, figsize=(14, 12))
        
        angles = ['Roll', 'Pitch', 'Yaw']
        
        # 定义颜色
        colors = {
            'ground_truth': '#1f77b4',      # 蓝色
            'pure_integration': '#ff7f0e',  # 橙色
            'kalman_filter': '#d62728',     # 红色
            'acceleration': '#2ca02c'       # 绿色
        }
        
        time_minutes = (times - times[0]) / 60.0  # 转换为分钟
        
        # 对yaw角进行解包，消除跳变
        gt_yaw_unwrapped = self.unwrap_angle(ground_truth[:, 2])
        pi_yaw_unwrapped = self.unwrap_angle(pure_integration[:, 2])
        kalman_yaw_unwrapped = self.unwrap_angle(kalman_orientation[:, 2])
        
        for i in range(3):
            if i == 0:  # Roll
                gt_data = ground_truth[:, 0]
                pi_data = pure_integration[:, 0]
                kalman_data = kalman_orientation[:, 0]
                acc_data = acceleration_orientation[:, 0]
            elif i == 1:  # Pitch
                gt_data = ground_truth[:, 1]
                pi_data = pure_integration[:, 1]
                kalman_data = kalman_orientation[:, 1]
                acc_data = acceleration_orientation[:, 1]
            else:  # Yaw - 使用解包后的数据
                gt_data = gt_yaw_unwrapped
                pi_data = pi_yaw_unwrapped
                kalman_data = kalman_yaw_unwrapped
                acc_data = acceleration_orientation[:, 2]  # 加速度计yaw为0
            
            # 绘制地面真值 - 实线
            axes[i].plot(time_minutes, np.degrees(gt_data), 
                        label='Ground Truth', 
                        color=colors['ground_truth'], 
                        linewidth=2.0)
            
            # 绘制纯积分估计值 - 实线
            axes[i].plot(time_minutes, np.degrees(pi_data), 
                        label='Pure Integration (Gyro only)', 
                        color=colors['pure_integration'], 
                        linestyle='-',
                        linewidth=1.5)
            
            # 绘制卡尔曼滤波估计值 
            axes[i].plot(time_minutes, np.degrees(kalman_data), 
                        label='Kalman Filter', 
                        color=colors['kalman_filter'], 
                        linestyle='-',
                        linewidth=1.5)
            
            # 绘制加速度计估计值 
            if i < 2:  # 只绘制roll和pitch，不绘制yaw
                axes[i].plot(time_minutes, np.degrees(acc_data), 
                            label='Acceleration-based', 
                            color=colors['acceleration'], 
                            linestyle='-',
                            linewidth=1.5)
            
            axes[i].set_ylabel(f'{angles[i]} (degrees)')
            axes[i].legend(loc='upper right')
            axes[i].grid(True, alpha=0.3)
        
        axes[2].set_xlabel('Time (minutes)')
        axes[0].set_title(f'IMU Orientation Estimation Methods Comparison - {self.dataset_date}')
        
        plt.tight_layout()
        
        # 保存到输出目录
        output_path = os.path.join(self.run_dir, f'methods_comparison_{self.dataset_date}.png')
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        
        # 非阻塞显示
        plt.show(block=False)
        plt.pause(0.1)
        
        # 绘制误差图
        self.plot_errors(pure_integration, acceleration_orientation, kalman_orientation, ground_truth, times)
        
        plt.ioff()

    def plot_errors(self, pure_integration, acceleration_orientation, kalman_orientation, ground_truth, times):
        """绘制误差图"""
        plt.ion()
        
        # 计算误差
        pi_errors = pure_integration - ground_truth
        kalman_errors = kalman_orientation - ground_truth
        acc_errors = acceleration_orientation - ground_truth
        
        # 角度误差需要特殊处理（周期性）
        pi_roll_errors = np.arctan2(np.sin(pi_errors[:, 0]), np.cos(pi_errors[:, 0]))
        pi_pitch_errors = np.arctan2(np.sin(pi_errors[:, 1]), np.cos(pi_errors[:, 1]))
        kalman_roll_errors = np.arctan2(np.sin(kalman_errors[:, 0]), np.cos(kalman_errors[:, 0]))
        kalman_pitch_errors = np.arctan2(np.sin(kalman_errors[:, 1]), np.cos(kalman_errors[:, 1]))
        acc_roll_errors = np.arctan2(np.sin(acc_errors[:, 0]), np.cos(acc_errors[:, 0]))
        acc_pitch_errors = np.arctan2(np.sin(acc_errors[:, 1]), np.cos(acc_errors[:, 1]))
        
        # 对于yaw，使用解包后的角度计算误差
        yaw_pi_unwrapped = self.unwrap_angle(pure_integration[:, 2])
        yaw_kalman_unwrapped = self.unwrap_angle(kalman_orientation[:, 2])
        yaw_gt_unwrapped = self.unwrap_angle(ground_truth[:, 2])
        pi_yaw_errors = yaw_pi_unwrapped - yaw_gt_unwrapped
        kalman_yaw_errors = yaw_kalman_unwrapped - yaw_gt_unwrapped
        
        time_minutes = (times - times[0]) / 60.0
        
        fig, axes = plt.subplots(3, 1, figsize=(14, 12))
        
        angles = ['Roll', 'Pitch', 'Yaw']
        pi_angle_errors = [pi_roll_errors, pi_pitch_errors, pi_yaw_errors]
        kalman_angle_errors = [kalman_roll_errors, kalman_pitch_errors, kalman_yaw_errors]
        acc_angle_errors = [acc_roll_errors, acc_pitch_errors, np.zeros_like(acc_roll_errors)]  # yaw误差为0
        
        # 定义颜色
        pi_color = '#ff7f0e'  # 橙色
        kalman_color = '#d62728'  # 红色
        acc_color = '#2ca02c'  # 绿色
        
        for i in range(3):
            # 绘制纯积分误差 - 实线
            axes[i].plot(time_minutes, np.degrees(pi_angle_errors[i]), 
                        color=pi_color, linewidth=1.5, label='Pure Integration Error')
            
            # 绘制卡尔曼滤波误差 - 虚线
            axes[i].plot(time_minutes, np.degrees(kalman_angle_errors[i]), 
                        color=kalman_color, linewidth=1.5,  label='Kalman Filter Error')
            
            # 绘制加速度计误差 - 点划线 (只绘制roll和pitch)
            if i < 2:
                axes[i].plot(time_minutes, np.degrees(acc_angle_errors[i]), 
                            color=acc_color, linewidth=1.5,  label='Acceleration-based Error')
            
            axes[i].set_ylabel(f'{angles[i]} Error (degrees)')
            axes[i].grid(True, alpha=0.3)
            axes[i].legend(loc='upper right')
        
        axes[2].set_xlabel('Time (minutes)')
        axes[0].set_title(f'IMU Orientation Estimation Errors - {self.dataset_date}')
        
        plt.tight_layout()
        
        # 保存到输出目录
        output_path = os.path.join(self.run_dir, f'methods_errors_{self.dataset_date}.png')
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        
        # 非阻塞显示
        plt.show(block=False)
        plt.pause(0.1)
        
        # 计算并显示误差统计
        self.calculate_error_stats(pi_angle_errors, kalman_angle_errors, acc_angle_errors, angles)
        
        plt.ioff()

    def calculate_error_stats(self, pi_errors, kalman_errors, acc_errors, angles):
        """计算并显示误差统计"""
        print("\n误差统计 (度):")
        print("=" * 70)
        print(f"{'角度':<10} {'方法':<25} {'RMSE':<10} {'MAE':<10} {'Std':<10}")
        print("-" * 70)
        
        for i, angle in enumerate(angles):
            pi_rmse = np.degrees(np.sqrt(np.mean(pi_errors[i]**2)))
            pi_mae = np.degrees(np.mean(np.abs(pi_errors[i])))
            pi_std = np.degrees(np.std(pi_errors[i]))
            
            kalman_rmse = np.degrees(np.sqrt(np.mean(kalman_errors[i]**2)))
            kalman_mae = np.degrees(np.mean(np.abs(kalman_errors[i])))
            kalman_std = np.degrees(np.std(kalman_errors[i]))
            
            if i < 2:  # 只计算roll和pitch的加速度计误差
                acc_rmse = np.degrees(np.sqrt(np.mean(acc_errors[i]**2)))
                acc_mae = np.degrees(np.mean(np.abs(acc_errors[i])))
                acc_std = np.degrees(np.std(acc_errors[i]))
            else:  # yaw的加速度计误差设为NaN
                acc_rmse = acc_mae = acc_std = float('nan')
            
            print(f"{angle:<10} {'Pure Integration':<25} {pi_rmse:<10.3f} {pi_mae:<10.3f} {pi_std:<10.3f}")
            print(f"{angle:<10} {'Kalman Filter':<25} {kalman_rmse:<10.3f} {kalman_mae:<10.3f} {kalman_std:<10.3f}")
            if i < 2:
                print(f"{angle:<10} {'Acceleration-based':<25} {acc_rmse:<10.3f} {acc_mae:<10.3f} {acc_std:<10.3f}")
            print("-" * 70)
    

    def align_timestamps_common(self, imu_times_list, gt_times, imu_data_list, gt_data):
        """对齐时间戳 - 确保所有方法使用相同的时间点"""
        # 找到所有IMU数据的时间范围交集
        common_start = max(times[0] for times in imu_times_list)
        common_end = min(times[-1] for times in imu_times_list)
        
        print(f"共同时间范围: {common_start:.2f} - {common_end:.2f} 秒")
        
        # 找到在共同时间范围内的地面真值数据点
        valid_gt_indices = np.where((gt_times >= common_start) & (gt_times <= common_end))[0]
        
        print(f"有效地面真值数据点数: {len(valid_gt_indices)}")
        
        aligned_imu_list = []
        aligned_gt = []
        aligned_times = []
        
        for gt_idx in valid_gt_indices:
            gt_time = gt_times[gt_idx]
            gt_orient = gt_data[gt_idx]
            
            # 检查所有IMU数据在这个时间点是否都有对应的数据
            valid_for_all = True
            aligned_imu_for_this_time = []
            
            for imu_times, imu_data in zip(imu_times_list, imu_data_list):
                # 找到最接近的IMU时间戳
                time_diffs = np.abs(imu_times - gt_time)
                closest_idx = np.argmin(time_diffs)
                
                # 检查时间差是否在容忍范围内
                if time_diffs[closest_idx] >= 0.05:  # 50ms容忍度
                    valid_for_all = False
                    break
                    
                aligned_imu_for_this_time.append(imu_data[closest_idx])
            
            if valid_for_all:
                aligned_imu_list.append(aligned_imu_for_this_time)
                aligned_gt.append(gt_orient)
                aligned_times.append(gt_time)
        
        # 将aligned_imu_list转换为每个方法的单独数组
        aligned_imu_separate = []
        for i in range(len(imu_data_list)):
            method_data = [aligned_imu_list[j][i] for j in range(len(aligned_imu_list))]
            aligned_imu_separate.append(np.array(method_data))
        
        print(f"成功对齐的共同数据点数: {len(aligned_times)}")
        
        return aligned_imu_separate, np.array(aligned_gt), np.array(aligned_times)

    def validate(self):
        """主验证函数"""
        print(f"开始验证数据集: {self.dataset_date}")
        print("方法: 纯陀螺仪积分、加速度计姿态、卡尔曼滤波")
        
        # 1. 加载IMU数据
        print("加载IMU数据...")
        imu_data = read_imu_basic(self.dataset_date)
        
        print(f"IMU数据形状: {imu_data.shape}")
        print(f"IMU数据时间范围: {imu_data[0, 0]:.2f} - {imu_data[-1, 0]:.2f} 秒")
        
        # 截断IMU数据
        imu_data, imu_times = self.truncate_data(imu_data, imu_data[:, 0])
        
        # 2. 加载地面真值
        print("加载地面真值...")
        gt_data = self.load_ground_truth()
        if gt_data is None:
            print("无法加载地面真值数据，跳过验证")
            return None
        
        # 截断地面真值数据
        gt_orientations, gt_times = self.truncate_data(
            self.gt_orientations, self.gt_timestamps
        )
        
        # 3. 检查坐标系一致性
        self.check_coordinate_system(imu_data)
        
        # 4. 计算三种姿态估计方法（注释掉改进纯积分）
        print("计算纯积分姿态...")
        pure_integration_orientations = self.calculate_pure_integration_orientation(imu_data)
        print(f"纯积分姿态数据形状: {pure_integration_orientations.shape}")
        
        # 注释掉改进纯积分
        # print("计算改进纯积分姿态...")
        # improved_integration_orientations = self.calculate_improved_pure_integration(imu_data)
        # print(f"改进纯积分姿态数据形状: {improved_integration_orientations.shape}")
        
        print("计算加速度计姿态...")
        acceleration_orientations = self.calculate_acceleration_orientation(imu_data)
        print(f"加速度计姿态数据形状: {acceleration_orientations.shape}")
        
        print("计算卡尔曼滤波姿态...")
        kalman_orientations = self.calculate_kalman_filter_orientation(imu_data)
        print(f"卡尔曼滤波姿态数据形状: {kalman_orientations.shape}")
        
        # 5. 使用新的对齐方法确保所有方法使用相同的时间点
        print("对齐时间戳（确保所有方法使用相同时间点）...")
        
        # 准备所有方法的时间戳和数据（注释掉改进纯积分）
        imu_times_list = [
            pure_integration_orientations[:, 0],
            # improved_integration_orientations[:, 0],  # 注释掉改进纯积分
            acceleration_orientations[:, 0],
            kalman_orientations[:, 0]  # 添加卡尔曼滤波
        ]
        
        imu_data_list = [
            pure_integration_orientations[:, 1:4],
            # improved_integration_orientations[:, 1:4],  # 注释掉改进纯积分
            acceleration_orientations[:, 1:4],
            kalman_orientations[:, 1:4]  # 添加卡尔曼滤波
        ]
        
        # 使用新的对齐方法
        aligned_data, aligned_gt, aligned_times = self.align_timestamps_common(
            imu_times_list, gt_times, imu_data_list, gt_orientations
        )
        
        # 解包对齐后的数据（注释掉改进纯积分）
        aligned_pi, aligned_acc, aligned_kalman = aligned_data
        
        if len(aligned_pi) == 0:
            print("错误: 无法对齐时间戳")
            return None
        
        print(f"对齐后的共同数据点数: {len(aligned_pi)}")
        
        # 6. 计算性能指标
        print("计算性能指标...")
        pi_metrics = self.calculate_metrics(aligned_pi, aligned_gt)
        # improved_metrics = self.calculate_metrics(aligned_improved, aligned_gt)  # 注释掉
        acc_metrics = self.calculate_metrics(aligned_acc, aligned_gt)
        kalman_metrics = self.calculate_metrics(aligned_kalman, aligned_gt)
        
        # 7. 绘制结果
        print("生成可视化结果...")
        self.plot_comparison(aligned_pi, aligned_acc, aligned_kalman, aligned_gt, aligned_times)
        
        # 8. 保存结果
        self.results = {
            'dataset': self.dataset_date,
            'start_ratio': self.start_ratio,
            'end_ratio': self.end_ratio,
            'pure_integration_metrics': pi_metrics,
            # 'improved_integration_metrics': improved_metrics,  # 注释掉
            'acceleration_metrics': acc_metrics,
            'kalman_filter_metrics': kalman_metrics,
            'samples_compared': len(aligned_pi),
            'aligned_times': aligned_times,
            'pure_integration': aligned_pi,
            # 'improved_integration': aligned_improved,  # 注释掉
            'acceleration': aligned_acc,
            'kalman_filter': aligned_kalman,
            'ground_truth': aligned_gt
        }
        
        return self.results
    
    def print_results(self):
        """打印验证结果"""
        if not self.results:
            print("没有可用的验证结果")
            return
        
        print("\n" + "="*60)
        print(f"多方法验证结果 - {self.results['dataset']}")
        print(f"时间范围: {self.results['start_ratio']*100:.0f}% - {self.results['end_ratio']*100:.0f}%")
        print("="*60)
        print(f"比较样本数: {self.results['samples_compared']}")
        
        # 纯积分结果
        print("\n纯积分角度误差 (度):")
        pi_metrics = self.results['pure_integration_metrics']
        for key, value in pi_metrics.items():
            if 'rmse' in key or 'mae' in key or 'std' in key:
                print(f"  {key}: {np.degrees(value):.3f}°")
        
        # 卡尔曼滤波结果
        print("\n卡尔曼滤波角度误差 (度):")
        kalman_metrics = self.results['kalman_filter_metrics']
        for key, value in kalman_metrics.items():
            if 'rmse' in key or 'mae' in key or 'std' in key:
                print(f"  {key}: {np.degrees(value):.3f}°")
        
        # 加速度计结果
        print("\n加速度计角度误差 (度):")
        acc_metrics = self.results['acceleration_metrics']
        for key, value in acc_metrics.items():
            if 'rmse' in key or 'mae' in key or 'std' in key:
                print(f"  {key}: {np.degrees(value):.3f}°")
        
        # 性能总结
        print("\n性能总结:")
        print(f"  纯积分 - Roll:  RMSE={np.degrees(pi_metrics['roll_rmse']):.3f}°, Pitch: RMSE={np.degrees(pi_metrics['pitch_rmse']):.3f}°, Yaw: RMSE={np.degrees(pi_metrics['yaw_rmse']):.3f}°")
        print(f"  卡尔曼滤波 - Roll: RMSE={np.degrees(kalman_metrics['roll_rmse']):.3f}°, Pitch: RMSE={np.degrees(kalman_metrics['pitch_rmse']):.3f}°, Yaw: RMSE={np.degrees(kalman_metrics['yaw_rmse']):.3f}°")
        print(f"  加速度计 - Roll: RMSE={np.degrees(acc_metrics['roll_rmse']):.3f}°, Pitch: RMSE={np.degrees(acc_metrics['pitch_rmse']):.3f}°")
        
        # 保存详细结果到CSV文件
        self.save_detailed_results()

    def save_detailed_results(self):
        """保存详细结果到CSV文件"""
        if not self.results:
            return
            
        filename = os.path.join(self.run_dir, f"methods_comparison_details_{self.dataset_date}.csv")
        data_to_save = np.column_stack((
            self.results['aligned_times'],
            np.degrees(self.results['pure_integration'][:, 0]),  # roll_pi
            np.degrees(self.results['pure_integration'][:, 1]),  # pitch_pi
            np.degrees(self.results['pure_integration'][:, 2]),  # yaw_pi
            np.degrees(self.results['kalman_filter'][:, 0]),  # roll_kalman
            np.degrees(self.results['kalman_filter'][:, 1]),  # pitch_kalman
            np.degrees(self.results['kalman_filter'][:, 2]),  # yaw_kalman
            np.degrees(self.results['acceleration'][:, 0]),  # roll_acc
            np.degrees(self.results['acceleration'][:, 1]),  # pitch_acc
            np.degrees(self.results['ground_truth'][:, 0]),  # roll_gt
            np.degrees(self.results['ground_truth'][:, 1]),  # pitch_gt
            np.degrees(self.results['ground_truth'][:, 2])   # yaw_gt
        ))
        
        header = "timestamp,roll_pure_deg,pitch_pure_deg,yaw_pure_deg,roll_kalman_deg,pitch_kalman_deg,yaw_kalman_deg,roll_acc_deg,pitch_acc_deg,roll_gt_deg,pitch_gt_deg,yaw_gt_deg"
        np.savetxt(filename, data_to_save, delimiter=',', header=header, comments='')
        print(f"详细结果已保存到: {filename}")
        
        # 保存性能指标
        metrics_filename = os.path.join(self.run_dir, f"methods_comparison_metrics_{self.dataset_date}.txt")
        with open(metrics_filename, 'w') as f:
            f.write(f"多方法验证结果 - {self.results['dataset']}\n")
            f.write(f"时间范围: {self.results['start_ratio']*100:.0f}% - {self.results['end_ratio']*100:.0f}%\n")
            f.write("="*60 + "\n")
            f.write(f"比较样本数: {self.results['samples_compared']}\n\n")
            
            f.write("纯积分角度误差 (度):\n")
            pi_metrics = self.results['pure_integration_metrics']
            for key, value in pi_metrics.items():
                if 'rmse' in key or 'mae' in key or 'std' in key:
                    f.write(f"  {key}: {np.degrees(value):.3f}°\n")
            
            f.write("\n卡尔曼滤波角度误差 (度):\n")
            kalman_metrics = self.results['kalman_filter_metrics']
            for key, value in kalman_metrics.items():
                if 'rmse' in key or 'mae' in key or 'std' in key:
                    f.write(f"  {key}: {np.degrees(value):.3f}°\n")
            
            f.write("\n加速度计角度误差 (度):\n")
            acc_metrics = self.results['acceleration_metrics']
            for key, value in acc_metrics.items():
                if 'rmse' in key or 'mae' in key or 'std' in key:
                    f.write(f"  {key}: {np.degrees(value):.3f}°\n")
        
        print(f"性能指标已保存到: {metrics_filename}")

def main():
    # 获取可用的数据集日期
    available_dates = get_available_dates()
    
    if not available_dates:
        print("没有找到可用的数据集")
        return
    
    print("可用的数据集日期:")
    for i, date in enumerate(available_dates):
        print(f"  {i+1}. {date}")
    
    # 选择要验证的数据集
    choice = input(f"选择要验证的数据集 (1-{len(available_dates)}): ")
    
    try:
        idx = int(choice) - 1
        selected_date = available_dates[idx]
    except (ValueError, IndexError):
        print("无效选择，使用第一个可用日期")
        selected_date = available_dates[0]
    
    # 获取时间范围参数
    print("\n请输入分析的时间范围 (0.0-1.0):")
    start_ratio = float(input("起始时间比例 (默认 0.0): ") or "0.0")
    end_ratio = float(input("结束时间比例 (默认 1.0): ") or "1.0")
    
    # 验证参数
    start_ratio = max(0.0, min(start_ratio, 1.0))
    end_ratio = max(0.0, min(end_ratio, 1.0))
    if start_ratio >= end_ratio:
        print("错误: 起始时间比例必须小于结束时间比例")
        return
    
    # 执行验证
    print(f"\n验证数据集: {selected_date}")
    print("使用三种方法: 纯陀螺仪积分、改进纯积分、加速度计姿态")
    validator = IMUPureIntegrationValidator(selected_date, start_ratio, end_ratio)
    results = validator.validate()
    
    if results:
        validator.print_results()
    
    print("\n" + "-"*50)
    
    # 等待用户输入后关闭所有图形
    input("按Enter键关闭所有图形并退出...")
    plt.close('all')

if __name__ == "__main__":
    main()
