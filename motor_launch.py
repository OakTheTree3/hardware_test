import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hardware_test',
            executable='cpp_motor',
            name='cpp_motor'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
