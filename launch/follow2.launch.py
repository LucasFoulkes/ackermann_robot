#!/usr/bin/env python3
"""Person following v2: lidar tracker + opennav_following server.

  ros2 launch ackermann_robot follow2.launch.py

No Nav2 planner/costmap in the loop -- the following server runs a smooth
control law straight at the tracked pose. Needs the drive stack running
(lidar, odom, cmd_vel_to_effort). Replaces follow.launch.py.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("ackermann_robot")
    record = LaunchConfiguration("record", default="true")
    return LaunchDescription([
        SetEnvironmentVariable("ROS_LOG_DIR", "/tmp/follow_log"),
        DeclareLaunchArgument("record", default_value=record,
                              description="record a diagnostic rosbag to /tmp"),
        ExecuteProcess(
            cmd=["bash", "-c",
                 "exec ros2 bag record -o /tmp/follow_bag_$(date +%H%M%S) "
                 "/scan /scan_filtered /scan_nav /people_poses /person_target "
                 "/odom /tf /tf_static /cmd_vel_follow /cmd_vel_nav "
                 "/filtered_dynamic_pose"],
            output="screen",
            condition=IfCondition(record),
        ),
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            name="scan_filter",
            output="screen",
            parameters=[os.path.join(share, "config", "scan_filter.yaml")],
            remappings=[("scan", "/scan"), ("scan_filtered", "/scan_filtered")],
        ),
        Node(
            package="ackermann_robot",
            executable="person_tracker",
            name="person_tracker",
            output="screen",
            parameters=[{"use_map_veto": False}],
            remappings=[("/scan", "/scan_filtered")],
        ),
        Node(
            package="opennav_following",
            executable="opennav_following",
            name="following_server",
            output="screen",
            parameters=[os.path.join(share, "config", "following.yaml")],
            remappings=[("cmd_vel", "/cmd_vel_follow")],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_following",
            output="screen",
            parameters=[{"autostart": True,
                         "node_names": ["following_server"]}],
        ),
        Node(
            package="ackermann_robot",
            executable="follow_client",
            name="follow_client",
            output="screen",
        ),
    ])
