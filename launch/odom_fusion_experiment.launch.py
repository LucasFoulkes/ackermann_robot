#!/usr/bin/env python3
"""RF2O + D435i IMU fusion experiment (robot_localization EKF).

One launch for the practical odom stack (no driver, no Nav2):

  robot_state_publisher  -> /tf_static (base_link, base_laser, d435i_*)
  c1.launch.py           -> /scan @ ~10 Hz
  rf2o.launch.py         -> /odom_rf2o only (publish_tf:=false)
  d435i.launch.py        -> /imu/data (Madgwick, use_mag:=false)
  imu_throttle           -> /imu/data_ekf @ 30 Hz (drops 200 Hz flood)
  ekf_node               -> /odom + odom->base_link TF @ 20 Hz

Compare in PlotJuggler: /odom_rf2o (raw) vs /odom (fused).

Requires: ros-jazzy-robot-localization, realsense2_camera, imu_filter_madgwick

If EKF warns about IMU frame_id, check:
  ros2 topic echo /imu/data --once   # header.frame_id must TF to base_link
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory("ackermann_robot")
    launch_dir = os.path.join(pkg, "launch")
    serial_port = LaunchConfiguration("serial_port", default="/dev/ttyUSB0")
    use_imu = LaunchConfiguration("use_imu", default="true")

    def include(name, launch_arguments=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
            launch_arguments=launch_arguments or [],
        )

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value=serial_port,
                              description="RPLIDAR C1 serial device"),
        DeclareLaunchArgument("use_imu", default_value=use_imu,
                              description="Start D435i + Madgwick (false = RF2O-only into EKF)"),
        include("robot_state_publisher.launch.py"),
        include("c1.launch.py", [("serial_port", serial_port)]),
        include("ekf_odom.launch.py", [("use_imu", use_imu)]),
    ])
