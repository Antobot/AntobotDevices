#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

import lifecycle_msgs.msg
import os
import subprocess


def generate_launch_description():
    driver_dir = os.path.join(get_package_share_directory('antobot_devices_lidar'), 'config', 'lslidar_cx.yaml')

    # --- Declare all expected launch arguments ---
    declare_namespace = DeclareLaunchArgument(
        'name_space', default_value='cx', description='Namespace of the lidar driver'
    )

    declare_frame_id = DeclareLaunchArgument(
        'frame_id', default_value='livox_frame', description='Frame ID for pointcloud'
    )

    declare_device_ip = DeclareLaunchArgument(
        'device_ip', default_value='192.168.1.200', description='Device IP of lidar'
    )

    declare_msop_port = DeclareLaunchArgument(
        'msop_port', default_value='2368', description='MSOP port'
    )

    declare_difop_port = DeclareLaunchArgument(
        'difop_port', default_value='2369', description='DIFOP port'
    )

    # --- Get LaunchConfiguration values ---
    name_space = LaunchConfiguration('name_space')
    frame_id = LaunchConfiguration('frame_id')
    device_ip = LaunchConfiguration('device_ip')
    msop_port = LaunchConfiguration('msop_port')
    difop_port = LaunchConfiguration('difop_port')

    driver_node = LifecycleNode(package='lslidar_driver',
                                namespace=name_space,
                                executable='lslidar_driver_node',
                                name='lslidar_driver_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[driver_dir,{
                                    'frame_id': frame_id,
                                    'device_ip': device_ip,
                                    'msop_port': msop_port,
                                    'difop_port': difop_port
                                }],
                                )
        
    return LaunchDescription([
        driver_node
    ])
