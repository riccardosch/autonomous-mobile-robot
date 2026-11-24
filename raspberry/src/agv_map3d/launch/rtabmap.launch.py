"""3D mapping with RTAB-Map: Kinect (colour + aligned depth), LiDAR, 2D SLAM pose.

    ros2 launch agv_map3d rtabmap.launch.py name:=casa3d

Runs on top of a launch that gives the TF map -> odom -> base_footprint and /scan
(agv_bringup robot.launch.py or nav.launch.py with kinect:=false: only one process
can own the Kinect, and this one needs it in `registered` mode).

Starts, all in the /rtabmap namespace except the driver:
  Kinect driver   (driver:=true) depth_mode registered (depth aligned to the colour image), colour on
  rgbd_sync       pairs colour, depth and camera_info into one RGB-D message
  rtabmap         3D map; database ~/maps/<name>.db; TF map3d -> map

Database: ~/maps/<name>.db (replaced at start unless keep:=true).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def nodes(context):
    name = LaunchConfiguration('name').perform(context)
    keep = LaunchConfiguration('keep').perform(context).lower() == 'true'
    database = os.path.join(os.path.expanduser('~/maps'), name + '.db')
    os.makedirs(os.path.dirname(database), exist_ok=True)
    config = PathJoinSubstitution([FindPackageShare('agv_map3d'), 'config', 'rtabmap.yaml'])

    kinect = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('kinect_v1_bringup'), 'launch', 'kinect.launch.py'])),
        launch_arguments={'depth_mode': 'registered', 'enable_rgb': 'true',
                          'enable_audio': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('driver')))

    sync = Node(
        package='rtabmap_sync', executable='rgbd_sync', name='rgbd_sync', namespace='rtabmap',
        output='screen',
        parameters=[{'approx_sync': True, 'qos': 2, 'qos_camera_info': 1}],
        remappings=[('rgb/image', '/kinect/rgb/image_raw'),
                    ('depth/image', '/kinect/depth_registered/image_raw'),
                    ('rgb/camera_info', '/kinect/rgb/camera_info')])

    rtabmap = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap', namespace='rtabmap',
        output='screen',
        parameters=[config, {'database_path': database}],
        remappings=[('scan', '/scan')],
        arguments=[] if keep else ['--delete_db_on_start'])

    return [kinect, sync, rtabmap]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='map3d',
                              description='Database name in ~/maps (without .db)'),
        DeclareLaunchArgument('driver', default_value='true',
                              description='Also start the Kinect driver (false if it already runs in registered mode)'),
        DeclareLaunchArgument('keep', default_value='false',
                              description='true: continue the existing database instead of replacing it'),
        OpaqueFunction(function=nodes),
    ])
