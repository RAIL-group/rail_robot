from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    EnvironmentVariable,
    TextSubstitution
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from pathlib import Path


def launch_setup(context, *args, **kwargs):
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    world_filepath_launch_arg = LaunchConfiguration('world_filepath')
    x_pos_launch_arg = LaunchConfiguration('x_pos')
    y_pos_launch_arg = LaunchConfiguration('y_pos')
    z_pos_launch_arg = LaunchConfiguration('z_pos')
    yaw_launch_arg = LaunchConfiguration('yaw')
    add_robot_only_launch_arg = LaunchConfiguration('add_robot_only')

    # Setup necessary environment variables for Gazebo, otherwise robot won't load in gazebo
    gz_model_path_env_var = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
            '/usr/share/gazebo-11/models/',
            ':',
            str(Path(
                FindPackageShare('rail_robot').perform(context)
            ).parent.resolve()),
        ]
    )
    gz_media_path_env_var = SetEnvironmentVariable(
        name='GAZEBO_MEDIA_PATH',
        value=[
            EnvironmentVariable('GAZEBO_MEDIA_PATH', default_value=''),
            ':',
            str(Path(
                FindPackageShare('rail_robot').perform(context)
            ).parent.resolve()),
        ]
    )

    gazebo_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ]),
        ]),
        launch_arguments={
            'world': world_filepath_launch_arg,
        }.items(),
        condition=UnlessCondition(add_robot_only_launch_arg),
    )

    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        namespace=robot_name_launch_arg,
        arguments=[
            '-entity', robot_name_launch_arg,
            '-topic', 'robot_description',
            '-x', x_pos_launch_arg,
            '-y', y_pos_launch_arg,
            '-z', z_pos_launch_arg,
            '-Y', yaw_launch_arg,
        ],
        output={'both': 'log'},
    )

    spawn_joint_state_broadcaster_node = Node(
        name='joint_state_broadcaster_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            '-c',
            f'{robot_name_launch_arg.perform(context)}/controller_manager',
            'joint_state_broadcaster',
        ],
    )

    spawn_camera_controller_node = Node(
        name='camera_controller_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            '-c',
            f'{robot_name_launch_arg.perform(context)}/controller_manager',
            'camera_controller',
        ],
    )

    spawn_diffdrive_controller_node = Node(
        name='diffdrive_controller_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            '-c',
            [LaunchConfiguration('robot_name'), TextSubstitution(
                text='/controller_manager')],
            'diffdrive_controller',
        ],
    )

    return [
        gz_model_path_env_var,
        gz_media_path_env_var,
        gazebo_launch_include,
        spawn_robot_node,
        spawn_joint_state_broadcaster_node,
        spawn_camera_controller_node,
        spawn_diffdrive_controller_node,
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value='robot',
            description='Namespace for the robot')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'world_filepath',
            default_value=PathJoinSubstitution([
                FindPackageShare('rail_robot'),
                'worlds',
                'floor.world',
            ]),
            description="File path to the Gazebo 'world' file to load.")
    )

    declared_arguments.append(
        DeclareLaunchArgument('x_pos',
                             default_value='0.0',
                             description='X position for spawning the robot')
    )

    declared_arguments.append(
        DeclareLaunchArgument('y_pos',
                             default_value='0.0',
                             description='Y position for spawning the robot')
    )

    declared_arguments.append(
        DeclareLaunchArgument('z_pos',
                             default_value='0.0',
                             description='Z position for spawning the robot')
    )

    declared_arguments.append(
        DeclareLaunchArgument('yaw',
                             default_value='0.0',
                             description='Yaw orientation for spawning the robot')
    )
    declared_arguments.append(
        DeclareLaunchArgument('add_robot_only',
                             default_value='false',
                             description='If true, only adds the robot to an already running Gazebo instance')
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)])
