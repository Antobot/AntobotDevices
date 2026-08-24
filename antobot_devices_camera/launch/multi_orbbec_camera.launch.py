from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from ament_index_python.packages import get_package_share_directory

import os
from pathlib import Path


ORBBEC_VENDOR_ID = '2bc5'
CAMERAS = (
    ('camera_front', '4-4'),
    ('camera_rear', '2-2'),
)


def find_connected_cameras(sysfs_root=Path('/sys/bus/usb/devices')):
    connected = []

    for camera_name, usb_port in CAMERAS:
        vendor_file = sysfs_root / usb_port / 'idVendor'
        try:
            vendor_id = vendor_file.read_text(encoding='ascii').strip().lower()
        except (FileNotFoundError, OSError):
            continue

        if vendor_id == ORBBEC_VENDOR_ID:
            connected.append((camera_name, usb_port))

    return connected


def generate_launch_description():
    logger = get_logger('multi_orbbec_camera')
    package_dir = get_package_share_directory('orbbec_camera')
    launch_file_dir = os.path.join(package_dir, 'launch')
    camera_launch_file = os.path.join(launch_file_dir, 'gemini_330_series.launch.py')

    connected_cameras = find_connected_cameras()
    device_num = len(connected_cameras)
    if device_num == 0:
        configured_ports = ', '.join(usb_port for _, usb_port in CAMERAS)
        logger.warning(f'No Orbbec camera found on configured USB ports: {configured_ports}')
        return LaunchDescription()

    actions = []
    for camera_name, usb_port in connected_cameras:
        logger.info(
            f'Found {camera_name} on USB {usb_port}; starting with device_num={device_num}'
        )
        camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch_file),
            launch_arguments={
                'camera_name': camera_name,
                'usb_port': usb_port,
                'device_num': str(device_num),
                'sync_mode': 'free_run',
                'color_qos': 'SENSOR_DATA',
                'depth_qos': 'SENSOR_DATA',
            }.items(),
        )
        actions.append(GroupAction([camera]))

    return LaunchDescription(actions)
