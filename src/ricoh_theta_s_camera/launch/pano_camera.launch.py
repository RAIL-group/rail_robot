import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name')

    pano_camera_node = Node(package='ricoh_theta_s_camera',
                        executable='pano_camera_node',  
                        name='pano_camera_node',
                        output='screen',
                        namespace=robot_name)
    
    return [
        pano_camera_node,
    ]

    
def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument('robot_name', default_value='robot', description='Namespace for the robot')
    )
    return launch.LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)])

