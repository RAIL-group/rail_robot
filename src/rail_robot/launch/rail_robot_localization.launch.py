# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def launch_setup(context, *args, **kwargs):
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    map_yaml_file_launch_arg = LaunchConfiguration('map')
    use_sim_time_launch_arg = LaunchConfiguration('use_sim_time')
    params_file_launch_arg = LaunchConfiguration('params_file')
    autostart_launch_arg = LaunchConfiguration('autostart')
    use_respawn_launch_arg = LaunchConfiguration('use_respawn')
    log_level_launch_arg = LaunchConfiguration('log_level')

    lifecycle_nodes = ['map_server', 'amcl']

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    # remappings = [('/tf', 'tf'),
    #               ('/tf_static', 'tf_static')]
    remappings = []

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time_launch_arg,
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file_launch_arg,
            root_key=robot_name_launch_arg,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    load_nodes = GroupAction(
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn_launch_arg,
                respawn_delay=2.0,
                parameters=[
                    configured_params,
                    {'yaml_filename': map_yaml_file_launch_arg},
                ],
                arguments=['--ros-args', '--log-level', log_level_launch_arg],
                remappings=remappings),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=use_respawn_launch_arg,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level_launch_arg],
                remappings=remappings),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level_launch_arg],
                parameters=[{'autostart': autostart_launch_arg},
                            {'node_names': lifecycle_nodes}])
        ]
    )

    return [
        stdout_linebuf_envvar,
        load_nodes
    ]

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument('robot_name',
                              default_value='robot',
                              description='Namespace for the robot')
    )
    declared_arguments.append(
        DeclareLaunchArgument('map_yaml_file',
                              default_value=PathJoinSubstitution([
                                  FindPackageShare('rail_robot'),
                                  'worlds',
                                  'floor_map.yaml'
                              ]),
                              description='Full path to map yaml file to load')
    )
    declared_arguments.append(
        DeclareLaunchArgument('use_sim_time',
                              default_value='false',
                              description='Use simulation (Gazebo) clock if true')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
                FindPackageShare('rail_robot'),
                'config',
                'nav2_params_official.yaml'
        ]),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    )
    declared_arguments.append(
        DeclareLaunchArgument('autostart',
                              default_value='true',
                              description='Automatically startup the nav2 stack.')
    )
    declared_arguments.append(
        DeclareLaunchArgument('use_respawn',
                              default_value='False',
                              description='Whether to respawn if a node crashes.')
    )
    declared_arguments.append(
        DeclareLaunchArgument('log_level',
                              default_value='info',
                              description='log level')
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)])
