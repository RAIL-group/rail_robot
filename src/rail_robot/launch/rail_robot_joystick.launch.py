import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    params_file_launch_arg = LaunchConfiguration('params_file')

    remappings = [('/cmd_vel', (robot_name_launch_arg, '/commands/velocity'))]

    load_nodes = GroupAction(
        actions=[
            Node(
                package='joy', executable='joy_node', name='joy_node',
                parameters=[{
                    'device_id': 0,
                    'deadzone': 0.3,
                    'autorepeat_rate': 20.0}]),
            Node(
                package='teleop_twist_joy', executable='teleop_node',
                name='teleop_twist_joy_node',
                parameters=[params_file_launch_arg],
                remappings=remappings)
        ]
    )

    return [
        load_nodes
    ]

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument(
        'robot_name',
        default_value='robot',
        description='Namespace for the robot')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('rail_robot'),
                'config',
                'joystick_params.yaml'
            ]),
            description='Path to the joystick configuration YAML file')
    )

    return launch.LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
