from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_joy',
            namespace='',
            executable='teleop-launch',
            name='sim'
        ),
        Node(
            package='roomba_oi',
            namespace='',
            executable='roomba_teleop_adapter',
            name='sim'
        )
    ])