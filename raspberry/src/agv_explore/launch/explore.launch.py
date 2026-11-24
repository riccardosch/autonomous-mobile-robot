"""Explore the house by itself and save the map.

    ros2 launch agv_explore explore.launch.py
    ros2 launch agv_explore explore.launch.py kinect:=false map_name:=casa

Starts agv_bringup/nav.launch.py (robot, SLAM, Kinect depth, Nav2) and the
explorer, which sends Nav2 goals to the frontiers of the map until none is
left. The map is then saved in ~/maps (.pgm/.yaml and slam_toolbox .posegraph).
Ctrl+C stops everything; the wheels stop 0.5 s later at most.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('agv_bringup'), 'launch', 'nav.launch.py'])),
        launch_arguments={'kinect': LaunchConfiguration('kinect')}.items())
    explorer = Node(
        package='agv_explore', executable='explorer', name='explorer', output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('agv_explore'), 'config', 'explorer.yaml']),
                    # Explicit types: launch turns '5' into an integer, and a parameter
                    # declared as double rejects it (the node aborts at start).
                    {'map_name': ParameterValue(LaunchConfiguration('map_name'), value_type=str),
                     'time_limit_min': ParameterValue(LaunchConfiguration('time_limit_min'),
                                                      value_type=float)}])
    return LaunchDescription([
        DeclareLaunchArgument('kinect', default_value='true'),
        DeclareLaunchArgument('map_name', default_value='',
                              description='Saved map name in ~/maps; empty = house_<date>_<time>'),
        DeclareLaunchArgument('time_limit_min', default_value='20.0'),
        nav,
        explorer,
    ])
