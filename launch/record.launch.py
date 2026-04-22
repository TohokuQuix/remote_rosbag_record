from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("remote_rosbag_record")
    default_config = str(Path(pkg_share) / "config" / "record_params.example.yaml")

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="Path to record_params.yaml",
    )

    output_dir_arg = DeclareLaunchArgument(
        "output_directory",
        default_value="",
        description="Directory where bags are saved (overrides config if non-empty)",
    )

    record_node = Node(
        package="remote_rosbag_record",
        executable="record",
        name="remote_rosbag_record",
        parameters=[
            LaunchConfiguration("config"),
            {"output_directory": LaunchConfiguration("output_directory")},
        ],
        output="screen",
    )

    return LaunchDescription([
        config_arg,
        output_dir_arg,
        record_node,
    ])
