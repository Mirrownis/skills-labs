from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='main',
            executable='info',
        ),
        Node(
            package='main',
            executable='log',
            parameters=[
                {'--disable-stdout-logs'}
            ],
        ),
        Node(
            package='main',
            executable='plan',
            parameters=[
                {'--disable-stdout-logs'}
            ],
        )
    ])
