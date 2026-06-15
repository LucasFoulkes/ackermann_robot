#!/usr/bin/env python3
"""RTAB-Map graph SLAM — drop-in replacement for slam.launch.py (slam_toolbox).

Owns /map + the map->odom TF + loop closure, consuming:
  * /odom               EKF-fused odometry (icp_odometry + gyro), reused as-is
  * /scan_slam          speckle-filtered lidar -> occupancy grid + proximity loops
  * D435i RGB-D         color + aligned depth -> VISUAL loop closure (the point of
                        switching: disambiguates look-alike corridors that pure
                        lidar scan-matching teleports through)

Params live in config/rtabmap.yaml (symlinked -> tune + restart, no rebuild).
slam_toolbox MUST be off when this runs (both would fight over map->odom);
navigation.launch.py gates that via slam:=rtabmap.

Inspect the map / loop closures live from a laptop:
  ros2 run rtabmap_viz rtabmap_viz
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# D435i topics: realsense node is namespace='camera' name='camera' -> /camera/camera/...
# (camera_name='d435i' only sets the TF frame prefix, not the topic namespace).
RGB_IMAGE = "/camera/camera/color/image_raw"
RGB_INFO = "/camera/camera/color/camera_info"
DEPTH_IMAGE = "/camera/camera/aligned_depth_to_color/image_raw"  # needs align_depth.enable=true


def generate_launch_description():
    pkg = get_package_share_directory("ackermann_robot")
    params = os.path.join(pkg, "config", "rtabmap.yaml")
    scan_topic = LaunchConfiguration("scan_topic", default="/scan_slam")
    odom_topic = LaunchConfiguration("odom_topic", default="/odom")

    return LaunchDescription([
        DeclareLaunchArgument("scan_topic", default_value="/scan_slam"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[params],
            remappings=[
                ("rgb/image", RGB_IMAGE),
                ("rgb/camera_info", RGB_INFO),
                ("depth/image", DEPTH_IMAGE),
                ("scan", scan_topic),
                ("odom", odom_topic),
            ],
            # "-d" = delete the database on start -> fresh map each run, like
            # slam_toolbox online_async. Drop it (and flip Mem/IncrementalMemory
            # false) to reuse a saved .db for localization-only later.
            arguments=["-d", "--ros-args", "--log-level", "rtabmap:=warn"],
        ),
    ])
