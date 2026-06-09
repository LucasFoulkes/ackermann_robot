#!/usr/bin/env python3
"""D435i depth -> floor-free LaserScan (RANSAC ground segmentation).

Brings up the D435i with depth + point cloud (via d435i.launch.py, which also
starts the IMU + Madgwick), then runs depth_floor_scan to:
  * publish /camera/scan      — obstacles only, floor removed, levelled
  * publish /camera/floor/distance — camera height above the floor (m)
  * publish /camera/floor/tilt     — camera roll/pitch/total tilt vs floor (deg)

Color is enabled here so the RealSense point-cloud filter has a texture stream
and publishes /camera/camera/depth/color/points. If you later switch the cloud
topic, override cloud_topic on the node.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    d435i_launch = os.path.join(
        get_package_share_directory("ackermann_robot"), "launch", "d435i.launch.py"
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(d435i_launch),
        launch_arguments={
            "enable_depth": "true",
            "enable_color": "true",
            "enable_pointcloud": "true",
        }.items(),
    )

    floor_scan_yaml = os.path.join(
        get_package_share_directory("ackermann_robot"), "config", "depth_floor_scan.yaml"
    )
    floor_scan = Node(
        package="ackermann_robot",
        executable="depth_floor_scan",
        name="depth_floor_scan",
        output="screen",
        parameters=[floor_scan_yaml],
    )

    return LaunchDescription([camera, floor_scan])
