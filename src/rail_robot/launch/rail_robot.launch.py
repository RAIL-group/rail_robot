import os

import ament_index_python.packages
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import yaml


def generate_launch_description():
    namespace_args = DeclareLaunchArgument(
        'namespace', default_value='robot', description='Namespace for the robot'
    )

    # running kobuki node
    share_dir = ament_index_python.packages.get_package_share_directory('kobuki_node')
    params_file = os.path.join(share_dir, 'config', 'kobuki_node_params.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']
    kobuki_ros_node = launch_ros.actions.Node(package='kobuki_node',
                                              executable='kobuki_ros_node',
                                              output='both',
                                              namespace=LaunchConfiguration('namespace'),
                                              parameters=[params])

    return launch.LaunchDescription([namespace_args, kobuki_ros_node])
