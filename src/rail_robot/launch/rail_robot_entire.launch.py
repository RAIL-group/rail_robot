import launch
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
    SetEnvironmentVariable,
    RegisterEventHandler,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    EnvironmentVariable,
    TextSubstitution
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from pathlib import Path
from launch.conditions import LaunchConfigurationEquals


def launch_setup(context, *args, **kwargs):
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    world_filepath_launch_arg = LaunchConfiguration('world_filepath')
    hardware_type_launch_arg = LaunchConfiguration('hardware_type')
    slam_mode_launch_arg = LaunchConfiguration('slam_mode')
    map_yaml_file_launch_arg = LaunchConfiguration('map')
    autostart_launch_arg = LaunchConfiguration('autostart')
    params_file_launch_arg = LaunchConfiguration('params_file')
    use_respawn_launch_arg = LaunchConfiguration('use_respawn')


    # Gazebo simulation
    rail_robot_description_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rail_robot'),
                'launch',
                'rail_robot_simulation.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': robot_name_launch_arg,
            'use_rviz': use_rviz_launch_arg,
            'world_filepath': world_filepath_launch_arg,
            'hardware_type': hardware_type_launch_arg,
        }.items(),
    )

    # SLAM
    rail_robot_slam_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rail_robot'),
                'launch',
                'rail_robot_slam.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': robot_name_launch_arg,
        }.items(),
        condition = LaunchConfigurationEquals(
            launch_configuration_name='slam_mode',
            expected_value='slam'
        )
    )

    # Navigation and Localization
    # Specify the actions
    localization_group = GroupAction([
        PushRosNamespace(
            namespace=robot_name_launch_arg),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('rail_robot'),
                'launch',
                'rail_robot_localization.launch.py'])),
            launch_arguments={'robot_name': robot_name_launch_arg,
                              'map_yaml_file': map_yaml_file_launch_arg,
                              'use_sim_time': 'true',
                              'autostart': autostart_launch_arg,
                              'params_file': params_file_launch_arg,
                              'use_respawn': use_respawn_launch_arg}.items()),
    ])

    navigation_group = GroupAction([
        PushRosNamespace(
            namespace=robot_name_launch_arg),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('rail_robot'),
                'launch',
                'rail_robot_navigation.launch.py'])),
            launch_arguments={'robot_name': robot_name_launch_arg,
                              'use_sim_time': 'true',
                              'autostart': autostart_launch_arg,
                              'params_file': params_file_launch_arg,
                              'use_respawn': use_respawn_launch_arg}.items()),
    ])

    return [
        rail_robot_description_launch_include,
        rail_robot_slam_launch_include,
        localization_group,
        navigation_group
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument('robot_name',
                              default_value='robot',
                              description='Namespace for the robot')
    )
    declared_arguments.append(
        DeclareLaunchArgument('use_rviz',
                              default_value='true',
                              description='Whether to launch RViz')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'world_filepath',
            default_value=PathJoinSubstitution([
                FindPackageShare('rail_robot'),
                'worlds',
                'floor.world',
            ]),
            description="the file path to the Gazebo 'world' file to load.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument('hardware_type',
                               choices=(
                                    'actual',
                                    'gz_classic',
                               ),
                              default_value='gz_classic',
                              description='Type of hardware interface to use')
    )
    declared_arguments.append(
        DeclareLaunchArgument('slam_mode',
                               choices=(
                                    'localization',
                                    'slam',
                               ),
                              default_value='slam',
                              description='Whether to run in localization or SLAM mode')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
        'map',
        description='Full path to map yaml file to load')
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
        DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes.')
    )

    return launch.LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)])
