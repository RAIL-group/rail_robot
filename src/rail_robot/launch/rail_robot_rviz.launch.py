import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def launch_setup(context, *args, **kwargs):
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    rviz_config_launch_arg = LaunchConfiguration('rviz_config')
    use_sim_time_launch_arg = LaunchConfiguration('use_sim_time')

    remappings = [('/goal_pose', 'goal_pose'),
                  ('/clicked_point', 'clicked_point'),
                  ('/initialpose', 'initialpose')]

    namespaced_rviz_config_file = ReplaceString(
            source_file=rviz_config_launch_arg,
            replacements={'<robot_namespace>': ('/', robot_name_launch_arg)})

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=robot_name_launch_arg,
        arguments=['-d', namespaced_rviz_config_file],
        parameters=[{
            'use_sim_time': use_sim_time_launch_arg,
        }],
        output={'both': 'log'},
        remappings=remappings
    )

    return [
        rviz2_node,
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument('robot_name',
                              default_value='robot',
                              description='Namespace for the robot')
    )
    declared_arguments.append(
        DeclareLaunchArgument('use_sim_time',
                              default_value='false',
                              description='Use simulation (Gazebo) clock if true')
    )
    declared_arguments.append(
        DeclareLaunchArgument('rviz_config',
                default_value=PathJoinSubstitution([
                FindPackageShare('rail_robot'),
                'config',
                'namespaced_rviz.rviz',
            ]),
            description='Rviz configuration file')
    )
    return launch.LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)])
