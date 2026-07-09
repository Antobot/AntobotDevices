from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    default_param_file = PathJoinSubstitution(
        [
            FindPackageShare("pointcloud_concatenator"),
            "config",
            "concatenator.param.yaml",
        ]
    )

    param_file = LaunchConfiguration("param_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "param_file",
                default_value=default_param_file,
                description="Path to the point cloud concatenator parameter file.",
            ),
            Node(
                package="pointcloud_concatenator",
                executable="pointcloud_concatenator_node",
                name="pointcloud_concatenator",
                output="screen",
                parameters=[param_file],
            ),
        ]
    )
