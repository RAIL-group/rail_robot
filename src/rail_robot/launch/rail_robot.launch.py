import launch
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name')

    kobuki_ros_node_parameter_file = ParameterFile(
        param_file=PathJoinSubstitution([
            FindPackageShare('rail_robot'),
            'config',
            'kobuki_node_params.yaml',
        ]),
        allow_substs=True,
    )

    slam_toolbox_parameter_file = ParameterFile(
        param_file=PathJoinSubstitution([
            FindPackageShare('rail_robot'),
            'config',
            'slam_toolbox_online_async.yaml',
        ]),
        allow_substs=True,
    )

    kobuki_ros_node = Node(package='kobuki_node',
                           executable='kobuki_ros_node',
                           name='kobuki_ros_node',
                           output='both',
                           namespace=robot_name,
                           parameters=[kobuki_ros_node_parameter_file])
    lidar_node = Node(package='sllidar_ros2',
                      executable='sllidar_node',
                      name='sllidar_node',
                      output='screen',
                      namespace=robot_name,
                      parameters=[{'channel_type': 'serial',
                         'serial_port': '/dev/rplidar',
                         'serial_baudrate': 115200,
                         'frame_id': (robot_name, '/laser_frame_link'),
                         'inverted': False,
                         'angle_compensate': False}])
    laser_static_transform_node = Node(package='tf2_ros',
                                       executable='static_transform_publisher',
                                       name='static_transform_publisher',
                                       output='screen',
                                       arguments=['0', '0', '0', '0', '0', '0',
                                                  (robot_name, '/base_footprint'),
                                                  (robot_name, '/laser_frame_link')])
    mapping_node = Node(package='slam_toolbox',
                        executable='async_slam_toolbox_node',
                        name='slam_toolbox',
                        output='screen',
                        namespace=robot_name,
                        parameters=[
                            slam_toolbox_parameter_file,
                            {'use_sim_time': True}
                        ],
                        remappings=[('/map', ('/', robot_name, '/map'))]
                        )

    return [
        kobuki_ros_node,
        lidar_node,
        laser_static_transform_node,
        mapping_node
    ]


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument('robot_name', default_value='robot', description='Namespace for the robot')
    )
    return launch.LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)])
