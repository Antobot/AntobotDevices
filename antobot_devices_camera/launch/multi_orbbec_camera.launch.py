from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_dir = get_package_share_directory('orbbec_camera')
    launch_file_dir = os.path.join(package_dir, 'launch')
    camera_launch_file = os.path.join(launch_file_dir, 'gemini_330_series.launch.py')

    camera_front = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_file),
        launch_arguments={
            'camera_name': 'camera_front',
            'usb_port': '4-4',
            'device_num': '2',
            'sync_mode': 'free_run',
            'color_qos': 'SENSOR_DATA',
            'depth_qos': 'SENSOR_DATA',
        }.items(),
    )

    camera_rear = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_file),
        launch_arguments={
            'camera_name': 'camera_rear',
            'usb_port': '2-2',
            'device_num': '2',
            'sync_mode': 'free_run',
            'color_qos': 'SENSOR_DATA',
            'depth_qos': 'SENSOR_DATA',
        }.items(),
    )

    return LaunchDescription([
        GroupAction([camera_front]),
        GroupAction([camera_rear]),
    ])
