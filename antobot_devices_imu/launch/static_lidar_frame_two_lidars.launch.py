# Copyright (c) 2021, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# # # Code Description:     Remap the lidar frames generated from the URDF to the final lidar frames used for pointcloud
# # #                       topics (front and back lidars). Used instead of imu_euler when no IMU compensation of the
# # #                       lidar frames is wanted.

# Contact: soyoung.kim@antobot.ai
# # # #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # publish front lidar frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_static_to_final_lidar_frame_front',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                       'laser_link_front_static', 'laser_link_front'],
        ),
        # publish back lidar frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_static_to_final_lidar_frame_back',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                       'laser_link_back_static', 'laser_link_back'],
        ),
    ])
