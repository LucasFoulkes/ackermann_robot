#!/usr/bin/env python3
"""Nav2 + live SLAM + Ackermann control (headless on the Pi).

Default: loop closure + IMU; SLAM on /scan_slam (8 Hz). Local costmap + rf2o use /scan.
use_floor_scan:=true (default): D435i RANSAC -> /camera/scan in local costmap.

Diagnostics: [system_stats] in this terminal.
RViz: nav2.rviz, Fixed Frame map, Nav2 Goal tool.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory("ackermann_robot")
    launch_dir = os.path.join(pkg, "launch")
    nav2_params = os.path.join(pkg, "config", "nav2_params.yaml")
    nav2_bt_xml = os.path.join(pkg, "config", "ackermann_nav_to_pose.xml")
    nav2_through_poses_bt_xml = os.path.join(
        pkg, "config", "ackermann_nav_through_poses.xml")
    effort_yaml = os.path.join(pkg, "config", "cmd_vel_to_effort.yaml")
    driver_yaml = os.path.join(pkg, "config", "ackermann_driver.yaml")
    floor_scan_yaml = os.path.join(pkg, "config", "depth_floor_scan.yaml")
    speckle_yaml = os.path.join(pkg, "config", "scan_speckle_filter.yaml")

    use_ekf = LaunchConfiguration("use_ekf").perform(context) == "true"
    use_imu = LaunchConfiguration("use_imu").perform(context) == "true"
    fuse_imu = LaunchConfiguration("fuse_imu").perform(context)  # "true"/"false" -> EKF imu A/B
    use_floor_scan = LaunchConfiguration("use_floor_scan").perform(context) == "true"
    depth_on = "true" if use_floor_scan else "false"
    color_on = LaunchConfiguration("enable_color").perform(context)

    serial_port = LaunchConfiguration("serial_port")
    closed_loop = LaunchConfiguration("closed_loop")
    log_stats = LaunchConfiguration("log_system_stats").perform(context) == "true"

    def include(name, launch_arguments=None, condition=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
            launch_arguments=launch_arguments or [],
            condition=condition,
        )

    actions = []
    slam_params = os.path.join(pkg, "config", "slam_toolbox_online_async.yaml")
    # 5 Hz (was 8): slam_toolbox gates by minimum_travel_distance anyway, and the
    # extra scans just piled up in its TF message filter during CPU spikes —
    # "queue is full" drops loosened the matcher prior right before the
    # 2026-06-11 parallel-hall teleport. 5 Hz still gives a scan every ~6 cm.
    slam_hz = 5.0
    # SLAM-only throttle; rf2o + local costmap keep full /scan.
    actions.append(Node(
        package="ackermann_robot",
        executable="scan_throttle",
        name="scan_throttle",
        output="screen",
        parameters=[{"hz": slam_hz, "in_topic": "/scan", "out_topic": "/scan_slam_pre"}],
    ))
    # Speckle filter on the SLAM feed only: isolated phantom returns were
    # getting baked into the map as permanent freckles. Costmaps (DenoiseLayer)
    # and ICP odometry keep the raw /scan.
    actions.append(Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="scan_speckle_filter",
        output="screen",
        parameters=[speckle_yaml],
        remappings=[("scan", "/scan_slam_pre"), ("scan_filtered", "/scan_slam")],
    ))
    if log_stats:
        actions.append(Node(
            package="ackermann_robot",
            executable="system_stats",
            name="system_stats",
            output="screen",
            parameters=[{"period_s": 5.0}],
        ))

    actions.extend([
        include("robot_state_publisher.launch.py"),
        include("c1.launch.py", [("serial_port", serial_port)]),
    ])
    if use_ekf:
        actions.append(include(
            "ekf_odom.launch.py",
            [
                ("use_imu", "true" if use_imu else "false"),
                ("fuse_imu", fuse_imu),
                ("enable_depth", depth_on),
                ("enable_color", color_on),  # 424x240x15 (see d435i.launch.py); floor scan itself is xyz-only
                ("enable_pointcloud", depth_on),
            ],
        ))
    else:
        actions.append(include("rf2o.launch.py"))
    # D435i depth for floor scan when EKF runs without IMU.
    if use_floor_scan and not use_imu:
        actions.append(include("d435i.launch.py", [
            ("enable_depth", "true"),
            ("enable_color", color_on),
            ("enable_pointcloud", "true"),
        ]))
    if use_floor_scan:
        actions.append(Node(
            package="ackermann_robot",
            executable="depth_floor_scan",
            name="depth_floor_scan",
            output="screen",
            parameters=[floor_scan_yaml],
            respawn=True,
            respawn_delay=2.0,
        ))
    actions.append(include("slam.launch.py", [("slam_params_file", slam_params)]))
    actions.extend([
        Node(
            package="ackermann_robot",
            executable="cmd_vel_to_effort",
            name="cmd_vel_to_effort",
            output="screen",
            parameters=[effort_yaml, {"closed_loop": closed_loop}],
        ),
        Node(
            package="ackermann_robot",
            executable="ackermann_driver",
            name="actuator_driver",
            output="screen",
            parameters=[driver_yaml],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[nav2_params],
            remappings=[("cmd_vel", "/cmd_vel_nav")],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_params],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[nav2_params],
            remappings=[("cmd_vel", "/cmd_vel_nav")],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[nav2_params, {
                "default_nav_to_pose_bt_xml": nav2_bt_xml,
                "default_nav_through_poses_bt_xml": nav2_through_poses_bt_xml,
            }],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[nav2_params],
        ),
    ])
    return actions


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port", default="/dev/ttyUSB0")
    closed_loop = LaunchConfiguration("closed_loop", default="true")
    use_ekf = LaunchConfiguration("use_ekf", default="true")
    use_imu = LaunchConfiguration("use_imu", default="true")

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value=serial_port),
        DeclareLaunchArgument("closed_loop", default_value=closed_loop),
        DeclareLaunchArgument("use_ekf", default_value=use_ekf),
        DeclareLaunchArgument("fuse_imu", default_value="true",
                              description="EKF fuses gyro yaw-rate; false=ICP-only (A/B test)"),
        DeclareLaunchArgument("use_imu", default_value=use_imu,
                              description="D435i IMU for EKF"),
        DeclareLaunchArgument(
            "use_floor_scan", default_value="true",
            description="D435i depth RANSAC -> /camera/scan for local costmap",
        ),
        # Default on: 5 s CPU/mem/temp samples are cheap and we keep needing them
        # to decide where to spend Pi headroom (see 2026-06-11 tuning).
        DeclareLaunchArgument("enable_color", default_value="true",
                              description="D435i RGB stream (424x240x15, ~5-10%% core); "
                                          "set false to reclaim CPU"),
        DeclareLaunchArgument("log_system_stats", default_value="true"),
        OpaqueFunction(function=_launch_setup),
    ])
