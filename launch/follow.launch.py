#!/usr/bin/env python3
"""Person following: lidar tracker + Nav2 follow glue, one command.

  ros2 launch ackermann_robot follow.launch.py                # armed right away
  ros2 launch ackermann_robot follow.launch.py enabled:=false # start disarmed

Needs the drive stack running (Nav2 + SLAM + odom). Runtime toggle:
  ros2 service call /follow/enable std_srvs/srv/SetBool "{data: false}"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enabled = LaunchConfiguration("enabled", default="true")
    return LaunchDescription([
        DeclareLaunchArgument("enabled", default_value=enabled,
                              description="start with following armed"),
        Node(
            package="ackermann_robot",
            executable="person_tracker",
            name="person_tracker",
            output="screen",
        ),
        Node(
            package="ackermann_robot",
            executable="person_follower",
            name="person_follower",
            output="screen",
            parameters=[{"start_enabled": enabled}],
        ),
    ])
