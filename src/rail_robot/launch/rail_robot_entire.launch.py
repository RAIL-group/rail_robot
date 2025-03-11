import launch
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
    SetLaunchConfiguration,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import LaunchConfigurationEquals


def launch_setup(context, *args, **kwargs):
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    world_filepath_launch_arg = LaunchConfiguration('world_filepath')
    hardware_type_launch_arg = LaunchConfiguration('hardware_type')
    use_sim_time_launch_arg = LaunchConfiguration('use_sim_time')
    map_yaml_file_launch_arg = LaunchConfiguration('map')
    autostart_launch_arg = LaunchConfiguration('autostart')
    params_file_launch_arg = LaunchConfiguration('params_file')
    use_respawn_launch_arg = LaunchConfiguration('use_respawn')

    SetLaunchConfiguration(
        'use_sim_time',
        value='true' if LaunchConfigurationEquals('hardware_type', 'gz_classic') else 'false'
    )

    # Robot description launch
    rail_robot_description_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rail_robot'),
                'launch',
                'rail_robot_description.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': robot_name_launch_arg,
            'use_rviz': use_rviz_launch_arg,
            'use_sim_time': use_sim_time_launch_arg,
            'hardware_type': hardware_type_launch_arg,
        }.items(),
    )

    # Physical Hardware
    rail_robot_hardware_launch_include =IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rail_robot'),
                'launch',
                'rail_robot_hardware.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': robot_name_launch_arg,
        }.items(),
        condition=LaunchConfigurationEquals('hardware_type', 'actual')
    )

    # GZ simulation
    rail_robot_simulation_launch_include = IncludeLaunchDescription(
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
        condition=LaunchConfigurationEquals('hardware_type', 'gz_classic')
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
        condition=LaunchConfigurationEquals('slam_mode', 'slam')
    )

    # Navigation and Localization
    rail_robot_navigation_group_include = GroupAction([
        PushRosNamespace(
            namespace=robot_name_launch_arg),

        # Localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('rail_robot'),
                'launch',
                'rail_robot_localization.launch.py'])),
            launch_arguments={'robot_name': robot_name_launch_arg,
                              'map_yaml_file': map_yaml_file_launch_arg,
                              'use_sim_time': use_sim_time_launch_arg,
                              'autostart': autostart_launch_arg,
                              'params_file': params_file_launch_arg,
                              'use_respawn': use_respawn_launch_arg}.items(),
            condition=LaunchConfigurationEquals('slam_mode', 'localization')
        ),

        # Navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('rail_robot'),
                'launch',
                'rail_robot_navigation.launch.py'])),
            launch_arguments={'robot_name': robot_name_launch_arg,
                              'use_sim_time': use_sim_time_launch_arg,
                              'autostart': autostart_launch_arg,
                              'params_file': params_file_launch_arg,
                              'use_respawn': use_respawn_launch_arg}.items()
        ),
    ])


    return [
        rail_robot_description_launch_include,
        rail_robot_simulation_launch_include,
        rail_robot_hardware_launch_include,
        rail_robot_slam_launch_include,
        rail_robot_navigation_group_include,
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
        DeclareLaunchArgument('use_sim_time',
                              default_value='false',
                              description='Use simulation time')
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
        default_value=' ',
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
