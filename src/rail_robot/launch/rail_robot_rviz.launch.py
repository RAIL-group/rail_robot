from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString
import yaml
import tempfile


def get_rviz_config_file(rviz_config_path, robot_names):
    with open(rviz_config_path, 'r') as file:
        config = yaml.safe_load(file)

    for robot_name in robot_names:
        tool_entry = [{"Class": "rviz_default_plugins/SetInitialPose",
                       "Topic": {
                           "Value": f"/{robot_name}/initialpose"
                       }},
                      {"Class": "rviz_default_plugins/SetGoal",
                       "Topic": {
                           "Value": f"/{robot_name}/goal_pose"
                       }},
                      ]
        config['Visualization Manager']['Tools'].extend(tool_entry)

    with tempfile.NamedTemporaryFile(delete=False, mode='w', newline='', suffix='.rviz') as tmpfile:
        yaml.safe_dump(config, tmpfile, default_flow_style=False)
        return tmpfile.name


def launch_setup(context, *args, **kwargs):
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    all_robot_names_launch_arg = LaunchConfiguration('all_robot_names')
    rviz_config_launch_arg = LaunchConfiguration('rviz_config')
    use_sim_time_launch_arg = LaunchConfiguration('use_sim_time')

    remappings = [('/goal_pose', 'goal_pose'),
                  ('/clicked_point', 'clicked_point'),
                  ('/initialpose', 'initialpose')]

    namespaced_rviz_config_file = ReplaceString(
        source_file=rviz_config_launch_arg,
        replacements={'<robot_namespace>': ('/', robot_name_launch_arg)})

    all_robot_names = all_robot_names_launch_arg.perform(context).split(',')
    rviz_config_file = get_rviz_config_file(namespaced_rviz_config_file.perform(context), all_robot_names)

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=robot_name_launch_arg,
        arguments=['-d', rviz_config_file],
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
        DeclareLaunchArgument(
            'robot_name',
            default_value='robot',
            description='Namespace for the robot')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'all_robot_names',
            default_value=LaunchConfiguration('robot_name'),
            description='Names of all robots separated by comma')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('rail_robot'),
                'config',
                'namespaced_rviz.rviz',
            ]),
            description='Rviz configuration file')
    )
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)])
