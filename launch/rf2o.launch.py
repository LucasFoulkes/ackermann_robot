#!/usr/bin/env python3
"""rf2o laser odometry — 2D scan-matching odometry from the RPLIDAR C1.

Subscribes to /scan and publishes nav_msgs/Odometry on /odom plus the
odom -> base_link TF. This is the robot's only odometry source for now ("see
how far rf2o gets us"): no encoders, no IMU fusion yet.

Needs the base_link -> base_laser TF, which comes from the URDF via
robot_state_publisher.launch.py — run that alongside this (the bringup launch
does). On its own this is just for odometry bring-up/testing.

MIGRATION NOTE: when we add IMU + robot_localization, flip odom_topic to
/odom_rf2o and publish_tf to false here, and let the EKF own /odom + the TF.
Two odometry publishers on the same /odom + TF would fight.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    laser_scan_topic = LaunchConfiguration("laser_scan_topic", default="/scan")
    odom_topic = LaunchConfiguration("odom_topic", default="/odom")
    publish_tf = LaunchConfiguration("publish_tf", default="true")

    rf2o = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        parameters=[{
            "laser_scan_topic": laser_scan_topic,
            "odom_topic": odom_topic,
            "publish_tf": publish_tf,
            "base_frame_id": "base_link",
            "odom_frame_id": "odom",
            "init_pose_from_topic": "",
            # C1 spins at 10 Hz; match it (running faster gains nothing).
            "freq": 10.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("laser_scan_topic", default_value=laser_scan_topic),
        DeclareLaunchArgument("odom_topic", default_value=odom_topic),
        DeclareLaunchArgument("publish_tf", default_value=publish_tf),
        rf2o,
    ])
