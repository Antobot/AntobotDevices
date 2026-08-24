# Copyright (c) 2021, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# # # Code Description:     Remap the lidar frame generated from the URDF to the final lidar frame used for pointcloud
# # #                       topics. Used instead of imu_euler when no IMU compensation of the lidar frame is wanted.

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
            name='lidar_static_to_final_lidar_frame',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                       'laser_link_front_static', 'laser_link_front'],
        ),
    ])
