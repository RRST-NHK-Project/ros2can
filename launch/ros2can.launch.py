import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ros2can'), 'config', 'ros2can.yaml')

    return LaunchDescription([
        Node(
            package='ros2can',
            executable='ros2can',
            name='ros2can_gui',
            output='screen',
            parameters=[config],
        ),
    ])
