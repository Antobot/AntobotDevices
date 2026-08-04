from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='antobot_devices_gps',
            namespace='/',
            executable='gps_manager',
        ),
        Node(
            package='antobot_devices_gps',
            namespace='/',
            executable='gps_integrity',
        ),
        Node(
            package='antobot_devices_gps',
            namespace='/',
            executable='gps_signal_health',
        ),
    ])
