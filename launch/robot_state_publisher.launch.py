#!/usr/bin/env python3
"""robot_state_publisher — publish the URDF as /robot_description and TF.

Without this, base_link -> base_laser is missing and rf2o silently sees scans
in a frame it can't transform to base_link. Include this from any launch file
that needs the static sensor TF (sensors.launch.py, navigation.launch.py).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('ackermann_robot'),
        'urdf', 'robot.urdf.xacro',
    )
    robot_description = xacro.process_file(urdf_path).toxml()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    return LaunchDescription([rsp])
