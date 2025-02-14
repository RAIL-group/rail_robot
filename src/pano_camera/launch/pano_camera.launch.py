import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import ament_index_python.packages

def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='robot', description='Namespace for the robot'
    )

    pano_node = Node(
        package='pano_camera',
        executable='pano_camera_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
    )

    return LaunchDescription([
        namespace_arg,
        pano_node
    ])
