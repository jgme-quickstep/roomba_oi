"""Launch file for Roomba bridge node and teleop adapter
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch teleop adapter
        Node(
            package='teleop_twist_joy',
            namespace='roomba',
            executable='teleop-launch',
            name='teleop_joy'
        ),
        # Launch roomba bridge adapter
        Node(
            package='roomba_oi',
            namespace='roomba',
            executable='roomba_oi_main',
            name='teleop'
        ),
        # Launch roomba bridge adapter
        Node(
            package='roomba_oi',
            namespace='roomba',
            executable='roomba_teleop_adapter',
            name='bridge'
        )
    ])