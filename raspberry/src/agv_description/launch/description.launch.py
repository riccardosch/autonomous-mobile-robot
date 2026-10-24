"""Publish the custom robot model using live wheel states or static preview."""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


# Empty overrides preserve the single set of defaults in the Xacro file.
DIMENSIONS = ('body_diameter', 'wheel_separation', 'wheel_diameter',
              'wheel_width', 'body_height', 'body_ground_clearance', 'imu_top_offset',
              'lidar_above_imu', 'lidar_x_offset')


def create_nodes(context):
    mappings = {}
    for name in DIMENSIONS:
        value = LaunchConfiguration(name).perform(context)
        if value:
            number = float(value)
            if not math.isfinite(number) or (name != 'lidar_x_offset' and (number < 0 or (number == 0 and name not in ('body_ground_clearance', 'imu_top_offset', 'lidar_above_imu')))):
                raise ValueError(f'{name} must be finite and positive (clearance and sensor offsets may be zero; lidar_x_offset is signed)')
            mappings[name] = value
    preview_text = LaunchConfiguration('preview').perform(context).lower()
    if preview_text not in ('true', 'false'):
        raise ValueError('preview must be true or false')
    preview = preview_text == 'true'
    source = os.path.join(get_package_share_directory('agv_description'), 'urdf', 'agv.urdf.xacro')
    description = xacro.process_file(source, mappings=mappings).toxml()
    nodes = [Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(description, value_type=str)}],
        remappings=[('joint_states', 'preview_joint_states' if preview else '/wheel_states')],
        output='screen',
    )]
    if preview:
        nodes.append(Node(
            package='joint_state_publisher', executable='joint_state_publisher',
            parameters=[{'robot_description': description}],
            remappings=[('joint_states', 'preview_joint_states')],
            output='screen',
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('preview', default_value='false',
                              description='Publish static wheel positions without hardware.'),
        *(DeclareLaunchArgument(name, default_value='',
                                description='Override in metres; empty uses Xacro default.')
          for name in DIMENSIONS),
        OpaqueFunction(function=create_nodes),
    ])
