# Copyright 2026 Riccardo Schiavo
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Kinect driver plus the perception nodes, with one command.

  ros2 launch kinect_v1_perception perception.launch.py
  ros2 launch kinect_v1_perception perception.launch.py yolo:=false
  ros2 launch kinect_v1_perception perception.launch.py driver:=false   # driver already running
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import AndSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    def node(executable, condition):
        return Node(package='kinect_v1_perception', executable=executable,
                    namespace=namespace, output='screen', condition=IfCondition(condition))

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('kinect_v1_bringup'), 'launch', 'kinect.launch.py'])),
        launch_arguments={'namespace': namespace,
                          'depth_mode': LaunchConfiguration('depth_mode')}.items(),
        condition=IfCondition(LaunchConfiguration('driver')))

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='kinect'),
        DeclareLaunchArgument('driver', default_value='true',
                              description='Also start the Kinect driver (kinect_v1_bringup)'),
        DeclareLaunchArgument('depth_mode', default_value='mm',
                              description="Driver depth mode: 'mm', 'registered' or 'off'"),
        DeclareLaunchArgument('sound', default_value='true',
                              description='Sound detection, direction and LED'),
        DeclareLaunchArgument('sound_type', default_value='true',
                              description='Kind of sound (YAMNet); needs sound:=true'),
        DeclareLaunchArgument('yolo', default_value='true',
                              description='Object detection on the RGB image (YOLO26)'),
        driver,
        node('sound_localizer', LaunchConfiguration('sound')),
        node('sound_classifier', AndSubstitution(LaunchConfiguration('sound'),
                                                 LaunchConfiguration('sound_type'))),
        node('yolo_detector', LaunchConfiguration('yolo')),
    ])
