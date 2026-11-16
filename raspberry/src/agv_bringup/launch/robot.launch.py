"""Start the whole robot, with or without SLAM.

    ros2 launch agv_bringup robot.launch.py              # wheels, LiDAR, model, odometry, SLAM
    ros2 launch agv_bringup robot.launch.py slam:=false  # no map: to test odometry and the LiDAR

What it starts:
  micro_ros_agent   bridge to the wheel ESP32: /wheel_states, /imu/data_raw, /wheel_cmd
  lidar_bridge      /scan
  agv_description   robot model -> TF base_footprint ... laser
  agv_odometry      /wheel_states -> /odom and TF odom -> base_footprint; /cmd_vel -> /wheel_cmd
  slam_toolbox      /scan + TF -> /map and TF map -> odom

Keep the robot still while it starts: opening the port resets the wheel
ESP32, which calibrates the gyro.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include(package, *path, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(package), *path])),
        launch_arguments=(arguments or {}).items(), condition=condition)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('wheels_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('wheels_baud', default_value='921600'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('slam', default_value='true',
                              description='Build a map with slam_toolbox'),

        Node(package='micro_ros_agent', executable='micro_ros_agent', output='screen',
             arguments=['serial', '--dev', LaunchConfiguration('wheels_port'),
                        '-b', LaunchConfiguration('wheels_baud')]),
        include('lidar_bridge', 'launch', 'lidar.launch.py',
                arguments={'port': LaunchConfiguration('lidar_port')}),
        # Also starts the robot model (agv_description).
        include('agv_odometry', 'launch', 'odometry.launch.py'),
        include('slam_toolbox', 'launch', 'online_async_launch.py',
                arguments={
                    'use_sim_time': 'false',
                    'slam_params_file': PathJoinSubstitution(
                        [FindPackageShare('agv_bringup'), 'config', 'slam.yaml']),
                },
                condition=IfCondition(LaunchConfiguration('slam'))),
    ])
