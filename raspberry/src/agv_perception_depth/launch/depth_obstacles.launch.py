"""Kinect depth -> obstacle and floor clouds for Nav2.

    ros2 launch agv_perception_depth depth_obstacles.launch.py

Needs the Kinect driver running in the /kinect namespace (kinect_v1_bringup).
Out: /kinect/depth_obstacles/points and /kinect/depth_obstacles/floor (base_footprint).
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare('agv_perception_depth'), 'config', 'depth_obstacles.yaml'])
    return LaunchDescription([
        Node(package='agv_perception_depth', executable='depth_obstacles', name='depth_obstacles',
             namespace='kinect', output='screen', parameters=[params]),
    ])
