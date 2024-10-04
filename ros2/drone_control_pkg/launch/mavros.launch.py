import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from launch.events.process.process_exited import ProcessExited
from launch.event_handlers.on_process_exit import OnProcessExit
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    pth_mavros_launcher = get_package_share_directory("drone_control_pkg")
    pth_param0 = pth_mavros_launcher + "/config/apm_params.yaml"
    pth_param1 = pth_mavros_launcher + "/config/apm_config.yaml"
    pth_param2 = pth_mavros_launcher + "/config/apm_pluginlists.yaml"
    log_level = LaunchConfiguration("log_level")

    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        output="screen",
        parameters=[pth_param0, pth_param1, pth_param2],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return [mavros_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("setup_drone", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )