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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("ackermann_robot")
    launch_dir = os.path.join(pkg, "launch")
    ekf_params = os.path.join(pkg, "config", "ekf_rf2o_imu.yaml")
    use_imu = LaunchConfiguration("use_imu", default="true")
    # A/B toggle: when false, point the EKF's imu0 at a dead topic so it fuses
    # ICP odometry ONLY (camera/IMU hardware still run). Lets us measure the
    # IMU's contribution to odometry without breaking icp (which needs the camera).
    fuse_imu = LaunchConfiguration("fuse_imu", default="true")
    imu0_topic = PythonExpression(
        ["'/imu/data_ekf' if '", fuse_imu, "' == 'true' else '/imu/_off'"])
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
        # the EKF owns odom->base_link.
        # NOTE: ResetCountdown 1 was a TRAP — one failed registration reset the
        # odom, leaving no motion guess, then rtabmap looped on "null guess"
        # (froze /odom_icp -> SLAM shoved map->odom to -32 m; 2026-06-11/13).
        # Fix is ResetCountdown 0 (never auto-reset) ALONE: with no resets the
        # previous-motion guess is always available, so the null-guess loop can't
        # form.
        # IMU-into-ICP ABANDONED (2026-06-13): tried feeding the gyro to icp to
        # fix phantom linear velocity in turns. Three failures in a row -- silent
        # no-op (wait_imu_to_init false), buffering latency (throttled imu), then
        # full-rate imu DID subscribe with no buffering BUT odom_v was no better
        # (0.30 straight / 0.52 turning vs cmd 0.20 -- if anything worse). The
        # phantom velocity is NOT an ICP-guess problem; needs proper calibration
        # (real speed vs odom) before more changes. Reverted to the simple stable
        # config: ResetCountdown 0 (no null-guess), GuessMotion default, no IMU.
        # The speed-PI yaw gate in cmd_vel_to_effort handles the throttle-cut
        # symptom downstream.
        Node(
            package="rtabmap_odom",
            executable="icp_odometry",
            name="icp_odometry",
            # "both": rtabmap logs via its own printf-style logger (ULogger),
            # which bypasses rclcpp log files — the 2026-06-11 19:54 null-guess
            # death left zero trace on disk. "both" tees it into launch.log.
            output="both",
            parameters=[{
                "frame_id": "base_link",
                "odom_frame_id": "odom",
                "publish_tf": False,
                "qos_scan": 2,
                "wait_for_transform": 0.2,
                # DESKEW EXPERIMENT REVERTED (2026-06-14): feeding the IMU to icp
                # with deskewing improved jitter (0.060 -> 0.039) BUT motion
                # collapsed -- odom_v 0.31 -> 0.15, 30%% fully stuck at high
                # throttle. Same IMU-into-icp area that's bitten us repeatedly.
                # Back to Kalman-only odom (moves fine). Re-try deskew later as a
                # careful standalone test (suspect: IMU integration corrupting the
                # velocity estimate, or icp stalling on deskew compute).
                # FilteringStrategy 1 = Kalman filter on the odom velocity output:
                # ICP scan-match gives a slightly different displacement each frame
                # -> ~30%% velocity jitter (odom_v ±0.12 on 0.42, odom_w sign-flips
                # going straight, 2026-06-14). The Kalman smooths it before the
                # robot_localization EKF. KalmanMeasurementNoise up from default
                # 0.01 -> 0.05 trusts the motion model a bit more (more smoothing);
                # lower it if odom feels laggy.
                # Icp/RangeMin 0.25 (2026-07-04 self-return audit): the C1
                # sees the robot's own mast as a ~34-point arc at 0.07 m —
                # points RIGIDLY ATTACHED to the sensor that match at zero
                # displacement in every scan pair, anchoring ICP toward "no
                # motion". Prime suspect for the historic blind-reverse and
                # sluggish-onset odometry behaviors.
                "args": ("--Reg/Force3DoF true --Odom/ResetCountdown 0 "
                         "--Odom/FilteringStrategy 1 --Odom/KalmanMeasurementNoise 0.1 "
                         "--Icp/RangeMin 0.25"),
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
            parameters=[ekf_params, {"imu0": imu0_topic}],
            remappings=[("odometry/filtered", "/odom")],
        ),
    ])
