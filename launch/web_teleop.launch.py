#!/usr/bin/env python3
"""Drive the robot from a phone/browser: driver + web teleop.

  web UI  ->  /ackermann/cmd_effort  ->  ackermann_driver  ->  motor + servo

No lidar, Nav2, or cmd_vel_to_effort. Open http://<pi-ip>:8080/ on your phone.

  ros2 launch ackermann_robot web_teleop.launch.py
  ros2 launch ackermann_robot web_teleop.launch.py port:=9000
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("ackermann_robot")
    driver_yaml = os.path.join(pkg, "config", "ackermann_driver.yaml")
    host = LaunchConfiguration("host", default="0.0.0.0")
    port = LaunchConfiguration("port", default="8080")

    return LaunchDescription([
        DeclareLaunchArgument("host", default_value=host,
                              description="HTTP bind address (0.0.0.0 = all interfaces)"),
        DeclareLaunchArgument("port", default_value=port,
                              description="HTTP port for the teleop page"),
        Node(
            package="ackermann_robot",
            executable="ackermann_driver",
            name="actuator_driver",
            output="screen",
            parameters=[driver_yaml],
        ),
        Node(
            package="ackermann_robot",
            executable="web_teleop",
            name="web_teleop",
            output="screen",
            parameters=[{"host": host, "port": port}],
        ),
    ])
