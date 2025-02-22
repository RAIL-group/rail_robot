import launch
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # robot_name_launch_arg = LaunchConfiguration('robot_name')

    kobuki_ros_node_parameter_file = ParameterFile(
        param_file=PathJoinSubstitution([
            FindPackageShare('rail_robot'),
            'config',
            'kobuki_node_params2.yaml',
        ]),
        allow_substs=True,
    )
    slam_toolbox_parameter_file = ParameterFile(
        param_file=PathJoinSubstitution([
            FindPackageShare('rail_robot'),
            'config',
            'slam_toolbox_online_async2.yaml',
        ]),
        allow_substs=True,
    )
    robot_description = Command([
            'sh -c "',
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([
                FindPackageShare('rail_robot'),
                'urdf',
                'rail_robot.urdf.xacro'
            ]), ' ',
            'robot_name:=', '',
            ' ',
            # Stripping comments from the URDF is necessary for gazebo_ros2_control to parse the
            # robot_description parameter override
            '| ', FindExecutable(name='perl'), ' -0777 -pe \'s/<!--.*?-->//gs\'"'
        ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # namespace=robot_name_launch_arg,
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
        output={'both': 'log'},
    )
    kobuki_ros_node = Node(package='kobuki_node',
                           executable='kobuki_ros_node',
                           name='kobuki_ros_node',
                           output='both',
                        #    namespace=robot_name_launch_arg,
                           parameters=[kobuki_ros_node_parameter_file])
    sllidar_node = Node(package='sllidar_ros2',
                      executable='sllidar_node',
                      name='sllidar_node',
                      output='screen',
                    #   namespace=robot_name_launch_arg,
                      parameters=[{'channel_type': 'serial',
                         'serial_port': '/dev/rplidar',
                         'serial_baudrate': 115200,
                        #  'frame_id': (robot_name_launch_arg, '/laser_frame_link'),
                         'frame_id': 'laser_frame_link',
                         'inverted': False,
                         'angle_compensate': True}])
    slam_toolbox_node = Node(package='slam_toolbox',
                        executable='async_slam_toolbox_node',
                        name='slam_toolbox',
                        output='screen',
                        # namespace=robot_name_launch_arg,
                        parameters=[
                            slam_toolbox_parameter_file,
                            {'use_sim_time': False}
                        ],
                        # remappings=[('/map', ('/', robot_name_launch_arg, '/map'))]
                        )

    return [
        # robot_state_publisher_node,
        kobuki_ros_node,
        sllidar_node,
        slam_toolbox_node
    ]


def generate_launch_description():
    declared_arguments = []
    # robot_name_launch_arg = DeclareLaunchArgument('robot_name', default_value='robot', description='Namespace for the robot')
    # declared_arguments.append(robot_name_launch_arg)
    return launch.LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)])
