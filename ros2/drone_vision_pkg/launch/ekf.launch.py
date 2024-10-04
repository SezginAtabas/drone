from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[
                    os.path.join(
                        get_package_share_directory("drone_vision_pkg"),
                        "config",
                        "ekf.yaml",
                    )
                ],
                remappings=[("odometry/filtered", "/ekf/global")],
            ),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[
                    os.path.join(
                        get_package_share_directory("drone_vision_pkg"),
                        "config",
                        "ekf.yaml",
                    )
                ],
                remappings=[("odometry/filtered", "/ekf/local")],
            ),
        ]
    )