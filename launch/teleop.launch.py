#!/usr/bin/env python3
"""Launch only what is needed to move: PCA9685 (servo + motors) and PS4 Bluetooth teleop."""

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory("ackermann_robot"), "config"
    )

    ackermann_driver_node = Node(
        package="ackermann_robot",
        executable="ackermann_driver",
        name="actuator_driver",
        output="screen",
        parameters=[os.path.join(config_dir, "ackermann_driver.yaml")],
    )

    ps4_teleop_node = Node(
        package="ackermann_robot",
        executable="ps4_teleop",
        name="ps4_teleop",
        output="screen",
    )

    return LaunchDescription([
        ackermann_driver_node,
        ps4_teleop_node,
    ])
