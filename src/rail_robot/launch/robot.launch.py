import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import launch_ros.descriptions
import xacro
from launch.substitutions import Command

def generate_launch_description():
    kobuki_xacro_file = os.path.join(get_package_share_directory('kobuki_description'), 'urdf', 'kobuki_standalone.urdf.xacro')
    robot_description_raw = xacro.process_file(kobuki_xacro_file).toxml()

    # configure the robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'my_bot',
                                '-x', '0.0',
                                '-y', '0.0',
                                '-z', '0.0',
                                '-R', '0.0',
                                '-P', '0.0',
                                '-Y', '0.0'],
                    output='screen')

    tf_footprint2base_cmd = Node(package='tf2_ros',
                                 executable='static_transform_publisher',
                                 output='screen',
                                 arguments=['0.0', '0.0', '0.0',
                                            '0.0', '0.0', '0.0',
                                            'base_link',
                                            'base_footprint'])

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        tf_footprint2base_cmd
    ])
