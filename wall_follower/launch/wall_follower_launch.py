from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_follower',
            executable='Driver',
            output='screen'
        ),
        Node(
            package='wall_follower',
            executable='follower',
            output='screen'
        )
    ])
