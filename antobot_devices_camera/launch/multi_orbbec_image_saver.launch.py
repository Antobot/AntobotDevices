from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='antobot_devices_orbbec_camera',
            executable='multi_orbbec_image_saver',
            name='multi_orbbec_image_saver',
            output='screen',
            parameters=[{
                'camera_front.color_topic': '/camera_front/color/image_raw',
                'camera_front.depth_topic': '/camera_front/depth/image_raw',
                'camera_rear.color_topic': '/camera_rear/color/image_raw',
                'camera_rear.depth_topic': '/camera_rear/depth/image_raw',
                'save_every_n': 1,
                'jpeg_quality': 90,
                'sync': False,
                'sync_slop_ms': 30.0,
                'queue_size': 10,
                'qos_depth': 5,
                'depth_float_to_mm': False,
            }],
        ),
    ])
