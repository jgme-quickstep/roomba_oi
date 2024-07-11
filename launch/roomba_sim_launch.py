""" Launch file for simulated Roomba environment with teleo-op adapter
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Simulated roomba
        Node(
            package='roomba_oi',
            namespace='',
            executable='roomba_oi_main_sim',
            name='roomba_bridge'
        ),
        # Joystick teleop
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('teleop_twist_joy'), 'launch', 'teleop-launch.py'),
                # launch_arguments={'joy_config': "xbox"}.items()
            )
        ),
        # Teleop adapter
        Node(
            package='roomba_oi',
            namespace='',
            executable='roomba_teleop_adapter',
            name='roomba_teleop'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('roomba_gz_sim'), 'launch', 'quickstep_room.launch.py'),
                # launch_arguments={'joy_config': "xbox"}.items()
            )
        )
    ])