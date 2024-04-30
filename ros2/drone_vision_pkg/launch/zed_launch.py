import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


default_xacro_path = os.path.join(
        get_package_share_directory("drone_vision_pkg"),
        'urdf',
        'drone_description.urdf.xacro'
)

zed_xacro_path = os.path.join(
    get_package_share_directory("zed_wrapper"),
    'urdf',
    'zed_descr.urdf.xacro'
)

default_config_common = os.path.join(
        get_package_share_directory("drone_vision_pkg"),
        'config',
        'zed_common.yaml'
)



def launch_setup(context, *args, **kwargs):
    
    # Launch configuration variables
    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')
    
    publish_urdf = LaunchConfiguration('publish_urdf')
    publish_tf = LaunchConfiguration('publish_tf')
    publish_map_tf = LaunchConfiguration('publish_map_tf')
    publish_imu_tf = LaunchConfiguration('publish_imu_tf') 
    camera_name_val = camera_name.perform(context)
    camera_model_val = camera_model.perform(context)
    
    if camera_name_val == '':
        camera_name_val = 'zed'
    
    ros_override_path = os.path.join(
        get_package_share_directory("drone_vision_pkg"),
        'config',
        camera_model_val + '.yaml'
    )

    zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('zed_wrapper'),
            '/launch/zed_camera.launch.py'
        ]),
        launch_arguments={
            'camera_name': camera_name_val,
            'camera_model': camera_model_val,
            'publish_urdf' : publish_urdf,
            'publish_tf' : publish_tf,
            'publish_map_tf' : publish_map_tf,
            'publish_imu_tf' : publish_imu_tf,
            'xacro_path' : default_xacro_path,
            'config_path' : default_config_common,
            'ros_params_override_path' : ros_override_path
        }.items(),
    )
    
    vision_pose_node = Node(
        package="drone_vision_pkg",
        executable="vision_pose_node",
        output="screen"
    )
    
    
    return [zed_wrapper_launch, vision_pose_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("camera_name", default_value=TextSubstitution(text='zed')),
        DeclareLaunchArgument(
            'config_path',
            default_value=TextSubstitution(text=default_config_common),
            description='Path to the YAML configuration file for the camera.'),
        DeclareLaunchArgument(
            'camera_model',
            description='[REQUIRED] The model of the camera. Using a wrong camera model can disable camera features.',
            choices=['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm']), 
        DeclareLaunchArgument(
            'publish_urdf',
            default_value='true',
            description='Enable URDF processing and starts Robot State Published to propagate static TF.',
            choices=['true', 'false']),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Enable publication of the `odom -> camera_link` TF.',
            choices=['true', 'false']),
        DeclareLaunchArgument(
            'publish_map_tf',
            default_value='true',
            description='Enable publication of the `map -> odom` TF. Note: Ignored if `publish_tf` is False.',
            choices=['true', 'false']),
        DeclareLaunchArgument(
            'publish_imu_tf',
            default_value='true',
            description='Enable publication of the IMU TF. Note: Ignored if `publish_tf` is False.',
            choices=['true', 'false']),
        DeclareLaunchArgument(
            'xacro_path',
            default_value=TextSubstitution(text=default_xacro_path),
            description='Path to the camera URDF file as a xacro file.'),  
        
        OpaqueFunction(function = launch_setup)
        ])