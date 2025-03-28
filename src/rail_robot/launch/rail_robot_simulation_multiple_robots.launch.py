import launch
from launch_ros.actions import PushRosNamespace
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import LaunchConfigurationEquals



def launch_setup(context, *args, **kwargs):
    robots = [
        {'name': 'robot1', 'x_pose': 0.0, 'y_pose': 0.0, 'z_pose': 0.00, 'yaw': 0.0},
        {'name': 'robot2', 'x_pose': 0.0, 'y_pose': 1.0, 'z_pose': 0.00, 'yaw': 0.0}]

    only_add_robot = 'false'

    robot_launch_instances = []
    for robot in robots:
        group = GroupAction([
            # Robot description
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('rail_robot'),
                    'launch',
                    'rail_robot_description.launch.py'])),
                launch_arguments={
                    'robot_name': robot['name'],
                    'use_sim_time': 'true',
                    'use_rviz': 'true',
                    'hardware_type': 'gz_classic'}.items(),
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('rail_robot'),
                        'launch',
                        'rail_robot_simulation.launch.py'
                    ])
                ]),
                launch_arguments={
                    'robot_name': robot['name'],
                    'x_pos': TextSubstitution(text=str(robot['x_pose'])),
                    'y_pos': TextSubstitution(text=str(robot['y_pose'])),
                    'z_pos': TextSubstitution(text=str(robot['z_pose'])),
                    'yaw': TextSubstitution(text=str(robot['yaw'])),
                    'add_robot_only': only_add_robot,
                }.items(),
            ),
        ])
        only_add_robot = 'true'
        robot_launch_instances.append(group)

    return robot_launch_instances

def generate_launch_description():
    declared_arguments = []
    return launch.LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)])
