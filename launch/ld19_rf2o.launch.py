#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ackermann_share_dir = get_package_share_directory("ackermann_robot")
    default_slam_params_file = os.path.join(
        ackermann_share_dir, "config", "slam_toolbox_online_async.yaml"
    )

    # LD19
    port_name = LaunchConfiguration("port_name")
    port_baudrate = LaunchConfiguration("port_baudrate")
    scan_topic = LaunchConfiguration("scan_topic")
    laser_scan_topic = LaunchConfiguration("laser_scan_topic")
    laser_frame = LaunchConfiguration("laser_frame")

    # TF
    base_frame = LaunchConfiguration("base_frame")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")

    # RF2O
    odom_frame = LaunchConfiguration("odom_frame")
    odom_topic = LaunchConfiguration("odom_topic")
    publish_tf = LaunchConfiguration("publish_tf")
    freq = LaunchConfiguration("freq")

    # SLAM Toolbox
    enable_slam_toolbox = LaunchConfiguration("enable_slam_toolbox")
    slam_params_file = LaunchConfiguration("slam_params_file")

    ldlidar_node = Node(
        package="ldlidar_stl_ros2",
        executable="ldlidar_stl_ros2_node",
        name="LD19",
        output="screen",
        parameters=[
            {"product_name": "LDLiDAR_LD19"},
            {"topic_name": scan_topic},
            {"frame_id": laser_frame},
            {"port_name": port_name},
            {"port_baudrate": port_baudrate},
            {"laser_scan_dir": True},
            {"enable_angle_crop_func": False},
            {"angle_crop_min": 135.0},
            {"angle_crop_max": 225.0},
        ],
    )

    base_link_to_laser_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_base_laser_ld19",
        arguments=[x, y, z, roll, pitch, yaw, base_frame, laser_frame],
        output="screen",
    )

    rf2o_node = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o",
        output="screen",
        arguments=['--ros-args', '--log-level', 'ERROR'],
        parameters=[
            {"laser_scan_topic": laser_scan_topic},
            {"odom_frame_id": odom_frame},
            {"base_frame_id": base_frame},
            {"odom_topic": odom_topic},
            {"publish_tf": publish_tf},
            {"freq": freq},
        ],
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("slam_toolbox"),
                "launch",
                "online_async_launch.py",
            )
        ),
        launch_arguments={
            # Always run with wall-time on real hardware.
            "use_sim_time": "false",
            "slam_params_file": slam_params_file,
        }.items(),
        condition=IfCondition(enable_slam_toolbox),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("port_name", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("port_baudrate", default_value="230400"),
            DeclareLaunchArgument("scan_topic", default_value="scan"),
            DeclareLaunchArgument("laser_scan_topic", default_value="/scan"),
            DeclareLaunchArgument("laser_frame", default_value="base_laser"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("odom_topic", default_value="/odom_rf2o"),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            DeclareLaunchArgument("freq", default_value="10.0"),
            DeclareLaunchArgument("x", default_value="0.2"),
            DeclareLaunchArgument("y", default_value="0"),
            DeclareLaunchArgument("z", default_value="0.18"),
            DeclareLaunchArgument("roll", default_value="0"),
            DeclareLaunchArgument("pitch", default_value="0"),
            DeclareLaunchArgument("yaw", default_value="0"),

            # SLAM Toolbox (optional)
            DeclareLaunchArgument("enable_slam_toolbox", default_value="true"),
            DeclareLaunchArgument(
                "slam_params_file", default_value=str(default_slam_params_file)
            ),

            ldlidar_node,
            base_link_to_laser_tf_node,
            rf2o_node,
            slam_toolbox_launch,
        ]
    )
