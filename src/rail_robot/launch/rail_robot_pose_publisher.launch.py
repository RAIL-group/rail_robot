from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    robot_name_launch_arg = LaunchConfiguration('robot_name')

    robot_pose_publisher = Node(
        package='rail_robot',
        executable='robot_pose_publisher.py',
        name='robot_pose_publisher',
        namespace=robot_name_launch_arg,
        parameters=[{'robot_name': robot_name_launch_arg}],
        output='log'
    )

    return [
        robot_pose_publisher
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument(
        'robot_name',
        default_value='robot',
        description='Namespace for the robot')
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
