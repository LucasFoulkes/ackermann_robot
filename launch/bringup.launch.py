#!/usr/bin/env python3
"""Full 2D bringup: lidar + URDF TF + rf2o odometry + closed-loop control.

The "see how far rf2o gets us" stack:

  RPLIDAR C1 (/scan) ─▶ rf2o ─▶ /odom + odom->base_link TF
                                      │
  teleop/Nav2 (/cmd_vel) ─▶ cmd_vel_to_effort ─▶ /ackermann/cmd_effort ─▶ ackermann_driver ─▶ PWM
                                  ▲ optional slow speed trim from /odom (closed_loop:=true)

For Nav2 goals use drive.launch.py (SLAM + Nav2 + this control stack).
Set closed_loop:=false for open-loop feedforward tests.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("ackermann_robot")
    launch_dir = os.path.join(pkg, "launch")
    closed_loop = LaunchConfiguration("closed_loop", default="false")

    def include(name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)))

    effort_yaml = os.path.join(pkg, "config", "cmd_vel_to_effort.yaml")
    cmd_vel_to_effort = Node(
        package="ackermann_robot",
        executable="cmd_vel_to_effort",
        name="cmd_vel_to_effort",
        output="screen",
        parameters=[effort_yaml, {"closed_loop": closed_loop}],
    )

    ackermann_driver = Node(
        package="ackermann_robot",
        executable="ackermann_driver",
        name="actuator_driver",
        output="screen",
        parameters=[os.path.join(pkg, "config", "ackermann_driver.yaml")],
    )

    return LaunchDescription([
        DeclareLaunchArgument("closed_loop", default_value=closed_loop),
        include("robot_state_publisher.launch.py"),
        include("c1.launch.py"),
        include("rf2o.launch.py"),
        cmd_vel_to_effort,
        ackermann_driver,
    ])
