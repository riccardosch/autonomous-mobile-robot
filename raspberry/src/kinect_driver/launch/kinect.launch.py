"""Start the Kinect driver with its parameter file.

    ros2 launch kinect_driver kinect.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("kinect_driver"), "config", "kinect_driver.yaml")

    return LaunchDescription([
        Node(
            package="kinect_driver",
            executable="kinect_driver_node",
            name="kinect",
            output="screen",
            parameters=[params],
        ),
    ])
