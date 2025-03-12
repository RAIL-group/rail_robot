import launch
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    use_sim_time_launch_arg = LaunchConfiguration('use_sim_time')
    hardware_type_launch_arg = LaunchConfiguration('hardware_type')


    robot_description = Command([
            'sh -c "',
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([
                FindPackageShare('rail_robot'),
                'urdf',
                'rail_robot.urdf.xacro'
            ]), ' ',
            'robot_name:=', robot_name_launch_arg, ' ',
            'hardware_type:=', hardware_type_launch_arg, ' ',
            # Stripping comments from the URDF is necessary for gazebo_ros2_control to parse the
            # robot_description parameter override
            '| ', FindExecutable(name='perl'), ' -0777 -pe \'s/<!--.*?-->//gs\'"'
        ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name_launch_arg,
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time_launch_arg,
        }],
        output={'both': 'log'},
    )



    return [
        robot_state_publisher_node,
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
        DeclareLaunchArgument('hardware_type',
                               choices=(
                                    'actual',
                                    'gz_classic',
                               ),
                              default_value='actual',
                              description='Type of hardware interface to use')
    )
    return launch.LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)])
