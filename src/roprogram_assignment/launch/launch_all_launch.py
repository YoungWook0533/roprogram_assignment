from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roprogram_assignment',
            executable='node1',
            name='node1'
        ),
        Node(
            package='roprogram_assignment',
            executable='node2',
            name='node2',
            arguments=['5', '10']
        )
    ])
