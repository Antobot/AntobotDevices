from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sync', default_value='false'),
        DeclareLaunchArgument('save_every_n', default_value='1'),
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
                'save_every_n': ParameterValue(LaunchConfiguration('save_every_n'), value_type=int),
                'jpeg_quality': 90,
                'sync': ParameterValue(LaunchConfiguration('sync'), value_type=bool),
                'sync_slop_ms': 30.0,
                'queue_size': 10,
                'qos_depth': 5,
                'log_interval_sec': 0,
                'depth_float_to_mm': False,
            }],
        ),
    ])
