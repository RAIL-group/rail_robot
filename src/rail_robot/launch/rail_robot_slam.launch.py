from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    use_sim_time_launch_arg = LaunchConfiguration('use_sim_time')

    slam_toolbox_parameter_file = ParameterFile(
        param_file=PathJoinSubstitution([
            FindPackageShare('rail_robot'),
            'config',
            'slam_toolbox_online_async.yaml',
        ]),
        allow_substs=True,
    )

    slam_toolbox_node = Node(package='slam_toolbox',
                             executable='async_slam_toolbox_node',
                             name='slam_toolbox',
                             output='screen',
                             namespace=robot_name_launch_arg,
                             parameters=[
                                 slam_toolbox_parameter_file,
                                 {'use_sim_time': use_sim_time_launch_arg}
                             ],
                             remappings=[
                                 ('/map', ('/', robot_name_launch_arg, '/map'))]
                             )

    return [
        slam_toolbox_node
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name', default_value='robot', description='Namespace for the robot')
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)])
