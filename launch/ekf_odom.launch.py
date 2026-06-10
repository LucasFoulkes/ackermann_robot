#!/usr/bin/env python3
"""Fused odometry: ICP laser odom + D435i IMU -> EKF -> /odom + odom->base_link TF.

rtabmap icp_odometry publishes /odom_icp only (no TF). EKF fuses laser odom +
gyro and owns /odom. (rf2o.launch.py kept in repo but no longer wired in.)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("ackermann_robot")
    launch_dir = os.path.join(pkg, "launch")
    ekf_params = os.path.join(pkg, "config", "ekf_rf2o_imu.yaml")
    use_imu = LaunchConfiguration("use_imu", default="true")
    enable_depth = LaunchConfiguration("enable_depth", default="false")
    enable_color = LaunchConfiguration("enable_color", default="false")
    enable_pointcloud = LaunchConfiguration("enable_pointcloud", default="false")

    def include(name, launch_arguments=None, condition=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
            launch_arguments=launch_arguments or [],
            condition=condition,
        )

    return LaunchDescription([
        DeclareLaunchArgument("use_imu", default_value=use_imu,
                              description="D435i + Madgwick (false = RF2O-only into EKF)"),
        DeclareLaunchArgument("enable_depth", default_value=enable_depth,
                              description="D435i depth (true when use_floor_scan)"),
        DeclareLaunchArgument("enable_color", default_value=enable_color),
        DeclareLaunchArgument("enable_pointcloud", default_value=enable_pointcloud),
        # RTAB-Map ICP odometry (scan-to-map) replaces rf2o (scan-to-scan):
        # ~15% of a core at full 10 Hz, drifts less. publish_tf stays false —
        # the EKF owns odom->base_link. Odom/ResetCountdown auto-recovers if
        # ICP ever gets lost instead of staying dead.
        Node(
            package="rtabmap_odom",
            executable="icp_odometry",
            name="icp_odometry",
            output="screen",
            parameters=[{
                "frame_id": "base_link",
                "odom_frame_id": "odom",
                "publish_tf": False,
                "qos_scan": 2,
                "wait_for_transform": 0.2,
                "args": "--Reg/Force3DoF true --Odom/ResetCountdown 1",
            }],
            remappings=[("scan", "/scan"), ("odom", "/odom_icp")],
            arguments=["--ros-args", "--log-level", "icp_odometry:=warn"],
        ),
        include("d435i.launch.py", condition=IfCondition(use_imu), launch_arguments=[
            ("enable_depth", enable_depth),
            ("enable_color", enable_color),
            ("enable_pointcloud", enable_pointcloud),
        ]),
        # C++ throttle (topic_tools): same 200->30 Hz job as the old Python
        # imu_throttle node but ~40% of a core cheaper (no per-message
        # C<->Python conversion at 200 Hz).
        Node(
            package="topic_tools",
            executable="throttle",
            name="imu_throttle",
            output="screen",
            condition=IfCondition(use_imu),
            arguments=["messages", "/imu/data", "30.0", "/imu/data_ekf"],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_params],
            remappings=[("odometry/filtered", "/odom")],
        ),
    ])
