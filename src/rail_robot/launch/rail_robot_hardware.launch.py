import launch
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_name_launch_arg = LaunchConfiguration('robot_name')

    kobuki_ros_node_parameter_file = ParameterFile(
        param_file=PathJoinSubstitution([
            FindPackageShare('rail_robot'),
            'config',
            'kobuki_node_params.yaml',
        ]),
        allow_substs=True,
    )
    
    kobuki_ros_node = Node(package='kobuki_node',
                           executable='kobuki_ros_node',
                           name='kobuki_ros_node',
                           output='both',
                           namespace=robot_name_launch_arg,
                           parameters=[kobuki_ros_node_parameter_file])

    sllidar_node = Node(package='sllidar_ros2',
                      executable='sllidar_node',
                      name='sllidar_node',
                      output='screen',
                      namespace=robot_name_launch_arg,
                      parameters=[{'channel_type': 'serial',
                         'serial_port': '/dev/rplidar',
                         'serial_baudrate': 115200,
                         'frame_id': (robot_name_launch_arg, '/laser_frame_link'),
                         'inverted': False,
                         'angle_compensate': True}])

    return [
        kobuki_ros_node,
        sllidar_node,
    ]


def generate_launch_description():
    declared_arguments = []
    robot_name_launch_arg = DeclareLaunchArgument('robot_name', default_value='robot', description='Namespace for the robot')
    declared_arguments.append(robot_name_launch_arg)
    return launch.LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)])
