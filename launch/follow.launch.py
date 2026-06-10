#!/usr/bin/env python3
"""Person following: lidar tracker + Nav2 follow glue, one command.

  ros2 launch ackermann_robot follow.launch.py                # armed right away
  ros2 launch ackermann_robot follow.launch.py enabled:=false # start disarmed

Needs the drive stack running (Nav2 + SLAM + odom). Runtime toggle:
  ros2 service call /follow/enable std_srvs/srv/SetBool "{data: false}"
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
    enabled = LaunchConfiguration("enabled", default="true")
    map_veto = LaunchConfiguration("map_veto", default="false")
    record = LaunchConfiguration("record", default="true")
    return LaunchDescription([
        # rcl writes each node's log file here (person_tracker_*.log,
        # person_follower_*.log) so failures can be read back after a run
        SetEnvironmentVariable("ROS_LOG_DIR", "/tmp/follow_log"),
        DeclareLaunchArgument("enabled", default_value=enabled,
                              description="start with following armed"),
        DeclareLaunchArgument("map_veto", default_value=map_veto,
                              description="reject candidates on mapped obstacles"),
        DeclareLaunchArgument("record", default_value=record,
                              description="record a diagnostic rosbag to /tmp"),
        # everything needed to replay a failed run offline
        ExecuteProcess(
            cmd=["bash", "-c",
                 "exec ros2 bag record -o /tmp/follow_bag_$(date +%H%M%S) "
                 "/scan /scan_filtered /scan_nav /camera/scan /camera/scan_nav "
                 "/people_poses /person_target /goal_update /follow/status "
                 "/odom /tf /tf_static /cmd_vel_nav /plan "
                 "/local_costmap/costmap /global_costmap/costmap /map"],
            output="screen",
            condition=IfCondition(record),
        ),
        # ScanShadowsFilter strips veiling points (range-discontinuity
        # artifacts at object edges) that look exactly like 3-5 point legs
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
            parameters=[{"use_map_veto": map_veto}],
            remappings=[("/scan", "/scan_filtered")],
        ),
        Node(
            package="ackermann_robot",
            executable="person_follower",
            name="person_follower",
            output="screen",
            parameters=[{"start_enabled": enabled}],
        ),
    ])
