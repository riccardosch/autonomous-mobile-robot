"""Start the LiDAR bridge with its parameter file.

    ros2 launch lidar_bridge lidar.launch.py [port:=/dev/ttyACM0]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("lidar_bridge"), "config", "lidar_bridge.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
        Node(
            package="lidar_bridge",
            executable="lidar_bridge_node",
            name="lidar_bridge",
            output="screen",
            parameters=[params, {"port": LaunchConfiguration("port")}],
        ),
    ])
