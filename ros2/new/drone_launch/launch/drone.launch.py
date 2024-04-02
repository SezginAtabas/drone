import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



zed_common_conf_path = os.path.join(
    get_package_share_directory('drone_launch'),
    'config',
    'zed_common.yaml',
)

aruco_loc_conf_path = os.path.join(
    get_package_share_directory('drone_launch'),
    'config',
    'aruco_loc.yaml'
)

xacro_path = os.path.join(
    get_package_share_directory('drone_launch'),
    'urdf',
    'drone_description.urdf.xacro'
)

def launch_setup(context, *args, **kwargs):
    
    launch_rviz = LaunchConfiguration('launch_rviz')
    camera_model = LaunchConfiguration('camera_model')
    aruco_gravity_alignment = LaunchConfiguration('gravity_alignment')
    publish_tf = LaunchConfiguration('publish_tf')
    publish_map_tf = LaunchConfiguration('publish_map_tf')
    
    zed_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('zed_aruco_localization'),
            '/launch/zed_aruco_loc.launch.py'
        ]),
        launch_arguments={
            'camera_model' : camera_model,
            'publish_tf' : publish_tf,
            'publish_map_tf' : publish_map_tf,
            'rviz' : launch_rviz,
            'gravity_alignment' : aruco_gravity_alignment,
            'config_path' : zed_common_conf_path,
            'config_path_aruco' : aruco_loc_conf_path,
            'xacro_path' :  xacro_path,
        }.items(),
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("drone_launch"), 'config', 'ekf.yaml')],
    )
    
    vision_pose_node = Node(
        package='vision_pose',
        executable='vision_pose_node'
    )
    
    
    return [zed_launch, ekf_node, vision_pose_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description="Start rviz.",
            choices=['true', 'false']
        ),
        DeclareLaunchArgument(
            'camera_model',
            description='[REQUIRED] The model of the camera. Using a wrong camera model can disable camera features.',
            choices=['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm']
        ),
        DeclareLaunchArgument(
            'gravity_alignment',
            default_value='false',
            description='Enable orientation alignment to the gravity vector. Note: if enabled the orientation of the markers must refer to Earth gravity.',
            choices=['true', 'false']
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='false',
            description='Enable publication of the `odom -> camera_link` TF.',
            choices=['true', 'false']),
        DeclareLaunchArgument(
            'publish_map_tf',
            default_value='false',
            description='Enable publication of the `map -> odom` TF. Note: Ignored if `publish_tf` is False.',
            choices=['true', 'false']),

    OpaqueFunction(function=launch_setup)
    ])



