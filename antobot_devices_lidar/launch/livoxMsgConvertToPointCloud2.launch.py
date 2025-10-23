import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():


    config_path = PathJoinSubstitution(
        [FindPackageShare("antobot_devices_lidar"), "config", "aviaMsgConvert.yaml"]
    )

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="antobot_devices_lidar",
                executable="antobot_devices_lidar_convert",
                name="antobot_devices_lidar",
                output="screen",
                parameters=[{"config_path": config_path}]
            ),

        ]
    )