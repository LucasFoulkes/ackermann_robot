#!/usr/bin/env python3
"""Fused odometry: RF2O + D435i IMU -> EKF -> /odom + odom->base_link TF.

RF2O publishes /odom_rf2o only (no TF). EKF fuses laser odom + gyro and owns /odom.
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
        include("rf2o.launch.py", [
            ("odom_topic", "/odom_rf2o"),
            ("publish_tf", "false"),
        ]),
        include("d435i.launch.py", condition=IfCondition(use_imu), launch_arguments=[
            ("enable_depth", enable_depth),
            ("enable_color", enable_color),
            ("enable_pointcloud", enable_pointcloud),
        ]),
        Node(
            package="ackermann_robot",
            executable="imu_throttle",
            name="imu_throttle",
            output="screen",
            condition=IfCondition(use_imu),
            parameters=[{"imu_hz": 30.0}],
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
