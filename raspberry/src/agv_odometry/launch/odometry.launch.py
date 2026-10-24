"""Robot model (live wheel states) plus wheel odometry and /cmd_vel handling."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    description = os.path.join(get_package_share_directory('agv_description'),
                               'launch', 'description.launch.py')
    params = os.path.join(get_package_share_directory('agv_odometry'), 'config', 'odometry.yaml')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(description)),
        Node(package='agv_odometry', executable='odometry_node', name='odometry',
             parameters=[params], output='screen'),
    ])
