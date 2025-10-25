from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    locations_yaml_file_launch_arg = LaunchConfiguration('locations_yaml_file')

    tf_frame_broadcast_service_node = Node(
        package='rail_robot',
        executable='tf_frame_broadcast_service.py',
        name='tf_frame_broadcast_service',
        parameters=[{'locations_yaml_file': locations_yaml_file_launch_arg}]
    )

    save_location_service_node = Node(
        package='rail_robot',
        executable='save_location_service.py',
        name='save_location_service',
        parameters=[{'locations_yaml_file': locations_yaml_file_launch_arg}]
    )

    return [
        tf_frame_broadcast_service_node,
        save_location_service_node,
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'locations_yaml_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('rail_robot'),
                'worlds',
                'floor_map_containers.yaml'
            ]),
            description='Full path to the container locations YAML file')
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
