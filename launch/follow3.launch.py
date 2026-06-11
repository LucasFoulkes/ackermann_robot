#!/usr/bin/env python3
"""Person following v3: leg_tracker_ros2 (Leigh ICRA'15, complete package)
+ enrollment shim + opennav_following.

  ros2 launch ackermann_robot follow3.launch.py

Replaces the hand-crafted person_tracker with the field-proven pipeline:
detect_leg_clusters (C++ RF detector) -> occupancy_grid_mapping (non-person
static grid) -> joint_leg_tracker (Munkres + Kalman + free-space veto)
-> enroll_shim (front-cone target choice) -> following server.
Needs the drive stack (lidar, odom, cmd_vel_to_effort).
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
    leg_share = get_package_share_directory("leg_tracker_ros2")
    forest = os.path.join(leg_share, "config",
                          "trained_leg_detector_res=0.33.yaml")
    record = LaunchConfiguration("record", default="true")
    return LaunchDescription([
        SetEnvironmentVariable("ROS_LOG_DIR", "/tmp/follow_log"),
        DeclareLaunchArgument("record", default_value=record,
                              description="record a diagnostic rosbag to /tmp"),
        ExecuteProcess(
            cmd=["bash", "-c",
                 "exec ros2 bag record -o /tmp/follow_bag_$(date +%H%M%S) "
                 "/scan /scan_filtered /scan_nav /people_poses /person_target "
                 "/people_tracked /detected_leg_clusters /local_map "
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
            package="leg_tracker_ros2",
            executable="detect_leg_clusters",
            name="detect_leg_clusters",
            output="screen",
            parameters=[{
                "forest_file": forest,
                "fixed_frame": "odom",
                "detection_threshold": 0.10,
                "cluster_dist_euclid": 0.13,
                "min_points_per_cluster": 3,   # C1 at 0.72 deg: ~5 pts/leg at 2 m
                "max_detect_distance": 5.0,
                "max_detected_clusters": -1,
            }],
            remappings=[("/scan", "/scan_filtered")],
        ),
        Node(
            package="leg_tracker_ros2",
            executable="occupancy_grid_mapping",
            name="occupancy_grid_mapping",
            output="screen",
            parameters=[{
                "fixed_frame": "odom",
                "base_frame": "base_link",
                "scan_topic": "/scan_filtered",
                # sllidar no-return beams = nothing there = free space
                "invalid_measurements_are_free_space": True,
            }],
            remappings=[("scan", "/scan_filtered")],
        ),
        Node(
            package="leg_tracker_ros2",
            executable="joint_leg_tracker.py",
            name="joint_leg_tracker",
            output="screen",
            parameters=[{
                "scan_topic": "/scan_filtered",
                "fixed_frame": "odom",
                "scan_frequency": 10.0,
                "publish_people_frame": "odom",
            }],
        ),
        Node(
            package="ackermann_robot",
            executable="enroll_shim",
            name="enroll_shim",
            output="screen",
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
