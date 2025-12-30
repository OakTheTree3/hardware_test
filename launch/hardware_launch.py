import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_dir = get_package_share_directory('hardware_test')

    #Get path to the sensor launch file
    rft_sensor_launch_path = os.path.join(package_dir, 'launch', 'rft_sensor_launch.py')
    cpp_motor_launch_path = os.path.join(package_dir, 'launch', 'motor_launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rft_sensor_launch_path)
        ),

        Node(
            package='hardware_test',
            executable='cpp_pid',
            name='cpp_pid'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cpp_motor_launch_path)
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
