from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    use_camera_launch_arg = LaunchConfiguration('use_camera')
    rs_camera_pointcloud_enable_launch_arg = LaunchConfiguration('rs_camera_pointcloud_enable')
    rs_camera_logging_level_launch_arg = LaunchConfiguration('rs_camera_logging_level')
    rs_camera_output_location_launch_arg = LaunchConfiguration('rs_camera_output_location')

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
    rs_camera_node = Node(
        condition=IfCondition(use_camera_launch_arg),
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace=robot_name_launch_arg,
        name='camera',
        parameters=[
            {
                'publish_tf': True,
                'pointcloud.enable': rs_camera_pointcloud_enable_launch_arg,
            },
            ParameterFile(
                param_file=PathJoinSubstitution([
                    FindPackageShare('rail_robot'),
                    'config',
                    'rs_camera.yaml'
                ]),
                allow_substs=True,
            ),
        ],
        output=rs_camera_output_location_launch_arg.perform(context),
        arguments=[
            '--ros-args', '--log-level', rs_camera_logging_level_launch_arg.perform(context)
        ],
        emulate_tty=True,
    )

    return [
        kobuki_ros_node,
        sllidar_node,
        rs_camera_node,
    ]


def generate_launch_description():
    declared_arguments = []
    robot_name_launch_arg = DeclareLaunchArgument(
        'robot_name', default_value='robot', description='Namespace for the robot')
    declared_arguments.append(robot_name_launch_arg)
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_camera',
            default_value='true',
            choices=('true', 'false'),
            description='if `true`, the RealSense camera nodes are launched.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rs_camera_pointcloud_enable',
            default_value='true',
            choices=('true', 'false'),
            description="enables the RealSense camera's pointcloud.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rs_rbg_camera_profile',
            default_value='640,480,30',
            description='profile for the rbg camera image stream, in `<width>,<height>,<fps>`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rs_depth_module_profile',
            default_value='640,480,30',
            description='profile for the depth module stream, in `<width>,<height>,<fps>`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rs_camera_logging_level',
            default_value='info',
            choices=('debug', 'info', 'warn', 'error', 'fatal'),
            description='set the logging level for the realsense2_camera launch include.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rs_camera_output_location',
            default_value='screen',
            choices=('screen', 'log'),
            description='set the logging location for the realsense2_camera launch include.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rs_camera_align_depth',
            default_value='false',
            choices=('true', 'false'),
            description=(
                'whether to publish topics with the depth stream aligned with the color stream.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rs_camera_initial_reset',
            default_value='false',
            choices=('true', 'false'),
            description=(
                'On occasions the RealSense camera is not closed properly and due to firmware '
                'issues needs to reset. If set to `true`, the device will reset prior to usage.'
            ),
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)])
