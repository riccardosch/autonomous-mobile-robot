"""Whole robot + SLAM + Nav2: send a goal from RViz and the robot drives there.

    ros2 launch agv_bringup nav.launch.py

    ros2 launch agv_bringup nav.launch.py kinect:=false   # LiDAR only

What it starts:
  robot.launch.py       wheels, LiDAR, model, odometry, slam_toolbox (map is built while driving)
  kinect (kinect:=true) Kinect driver (kinect_v1_bringup, from ~/kinect) + depth_obstacles:
                        depth image -> obstacle and floor clouds for the costmaps
  planner_server        path from the robot to the goal on the global costmap (map + scan)
  controller_server     follows the path, publishes /cmd_vel (Regulated Pure Pursuit)
  behavior_server       recoveries: back up, wait, spin
  bt_navigator          the /navigate_to_pose action: plan, follow, recover
                        (behavior_trees/navigate_to_pose_agv.xml)
  lifecycle_manager     configures and activates the four servers in order

Only the Nav2 servers we need are started (no smoother, velocity smoother,
collision monitor, waypoint follower, docking): agv_odometry already ramps
the wheel speeds. Parameters: config/nav2.yaml.

Do not run teleop_twist_keyboard at the same time: both publish /cmd_vel.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# (package, executable); the node name is the executable, as in nav2.yaml.
SERVERS = [
    ('nav2_controller', 'controller_server'),
    ('nav2_planner', 'planner_server'),
    ('nav2_behaviors', 'behavior_server'),
    ('nav2_bt_navigator', 'bt_navigator'),
]


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare('agv_bringup'), 'config', 'nav2.yaml'])
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('agv_bringup'), 'launch', 'robot.launch.py'])),
        launch_arguments={
            'wheels_port': LaunchConfiguration('wheels_port'),
            'lidar_port': LaunchConfiguration('lidar_port'),
            'slam': 'true',
        }.items())

    kinect = IfCondition(LaunchConfiguration('kinect'))
    kinect_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('kinect_v1_bringup'), 'launch', 'kinect.launch.py'])),
        launch_arguments={'enable_rgb': LaunchConfiguration('kinect_rgb')}.items(),
        condition=kinect)
    depth_obstacles = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('agv_perception_depth'), 'launch',
                                  'depth_obstacles.launch.py'])),
        condition=kinect)

    # Our behavior tree (recovery: back up and look again with the Kinect).
    bt_xml = PathJoinSubstitution(
        [FindPackageShare('agv_bringup'), 'behavior_trees', 'navigate_to_pose_agv.xml'])
    nav2 = [Node(package=package, executable=name, name=name, output='screen',
                 parameters=[params] + ([{'default_nav_to_pose_bt_xml': bt_xml}]
                                        if name == 'bt_navigator' else []))
            for package, name in SERVERS]
    manager = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                   name='lifecycle_manager_navigation', output='screen', parameters=[params])

    return LaunchDescription([
        DeclareLaunchArgument('wheels_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('kinect', default_value='true',
                              description='Kinect depth in the costmaps (needs ~/kinect sourced)'),
        DeclareLaunchArgument('kinect_rgb', default_value='false',
                              description='Also publish the Kinect RGB image (not needed by Nav2)'),
        robot,
        kinect_driver,
        depth_obstacles,
        *nav2,
        manager,
    ])
