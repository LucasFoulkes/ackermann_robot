#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ackermann_share_dir = get_package_share_directory("ackermann_robot")

    # Include the existing LD19 + RF2O + SLAM launch file
    ld19_rf2o_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ackermann_share_dir, "launch", "localization.launch.py")
        )
    )

    # Ackermann Driver Node
    ackermann_driver_node = Node(
        package="ackermann_robot",
        executable="ackermann_driver",
        name="ackermann_driver",
        output="screen",
        # You can add parameters here if needed, e.g.:
        # parameters=[{"pwm_hz": 50.0}]
    )

    # Cmd Vel Bridge Node
    cmd_vel_bridge_node = Node(
        package="ackermann_robot",
        executable="cmd_vel_bridge",
        name="cmd_vel_bridge",
        output="screen",
    )

    return LaunchDescription([
        ld19_rf2o_launch,
        ackermann_driver_node,
        cmd_vel_bridge_node,
    ])
