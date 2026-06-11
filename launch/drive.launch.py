#!/usr/bin/env python3
"""Drive to goal: live SLAM + Nav2 + motor control (main outdoor stack).

  C1 /scan -> icp_odometry + IMU -> EKF -> /odom  (use_ekf:=false skips EKF)
  slam_toolbox -> /map + map->odom
  Nav2 -> cmd_vel_to_effort (deadband yaml) -> driver

On the Pi (default — SLAM, loop closure, EKF+IMU, D435i floor scan):
  ros2 launch ackermann_robot drive.launch.py

Disable depth floor scan only:
  ros2 launch ackermann_robot drive.launch.py use_floor_scan:=false

On a laptop (same ROS_DOMAIN_ID), RViz:
  rviz2 -d $(ros2 pkg prefix ackermann_robot)/share/ackermann_robot/rviz/nav2.rviz
  Use "Nav2 Goal" (not bare "2D Goal Pose").

Save map when done:
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: 'my_map'}}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("ackermann_robot")
    nav_launch = os.path.join(pkg, "launch", "navigation.launch.py")
    nav = LaunchConfiguration("nav", default="true")

    serial_port = LaunchConfiguration("serial_port", default="/dev/ttyUSB0")
    closed_loop = LaunchConfiguration("closed_loop", default="true")
    use_ekf = LaunchConfiguration("use_ekf", default="true")
    use_imu = LaunchConfiguration("use_imu", default="true")
    use_floor_scan = LaunchConfiguration("use_floor_scan", default="true")
    log_csv = LaunchConfiguration("log_csv", default="true")

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value=serial_port,
                              description="RPLIDAR C1 USB serial device"),
        DeclareLaunchArgument("closed_loop", default_value=closed_loop,
                              description="cmd_vel_to_effort odom speed trim"),
        DeclareLaunchArgument("use_ekf", default_value=use_ekf,
                              description="RF2O+IMU EKF on /odom"),
        DeclareLaunchArgument("use_imu", default_value=use_imu,
                              description="D435i IMU for EKF"),
        DeclareLaunchArgument("use_floor_scan", default_value=use_floor_scan,
                              description="D435i RANSAC -> /camera/scan in local costmap"),
        DeclareLaunchArgument("nav", default_value=nav,
                              description="start Nav2 (planner/costmaps); not needed for follow2/follow3"),
        DeclareLaunchArgument("log_csv", default_value=log_csv,
                              description="CSV telemetry (one file per run in ~/ros2_ws/logs)"),
        Node(
            package="ackermann_robot",
            executable="drive_logger",
            name="drive_logger",
            output="screen",
            condition=IfCondition(log_csv),
        ),
        # republishes /scan + /camera/scan with the followed person's beams
        # NaN'd out so the costmaps never see them as an obstacle
        Node(
            package="ackermann_robot",
            executable="scan_mask",
            name="scan_mask",
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch),
            condition=IfCondition(nav),
            launch_arguments={
                "serial_port": serial_port,
                "closed_loop": closed_loop,
                "use_ekf": use_ekf,
                "use_imu": use_imu,
                "use_floor_scan": use_floor_scan,
            }.items(),
        ),
    ])
