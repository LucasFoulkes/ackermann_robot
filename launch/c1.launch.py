#!/usr/bin/env python3
"""RPLIDAR C1 driver — drop-in replacement for ld19.launch.py.

Publishes sensor_msgs/LaserScan on /scan in frame base_laser.

The base_link -> base_laser transform is NOT published here: it comes from
robot_state_publisher (the URDF) started by sensors.launch.py. Run this file
on its own only for driver bring-up/testing, not for the full stack.

The C1 is connected via a CP2102N USB-UART adapter at 460800 baud.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port", default="/dev/ttyUSB0")
    serial_baudrate = LaunchConfiguration("serial_baudrate", default="460800")
    frame_id = LaunchConfiguration("frame_id", default="base_laser")
    # Flip if slam_toolbox builds a mirrored map (RPLIDAR scans CW by default).
    inverted = LaunchConfiguration("inverted", default="false")

    # RPLIDAR C1 publisher node (publishes topic scan)
    # respawn: the C1 intermittently fails its first handshake over the CP2102
    # USB-UART (SL_RESULT_OPERATION_TIMEOUT, exit 255). Auto-restart so it retries
    # until it spins up, instead of staying dead and leaving rf2o waiting forever.
    sllidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        output="screen",
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            "channel_type": "serial",
            "serial_port": serial_port,
            "serial_baudrate": serial_baudrate,
            "frame_id": frame_id,
            "inverted": inverted,
            "angle_compensate": True,
            "scan_mode": "Standard",
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value=serial_port),
        DeclareLaunchArgument("serial_baudrate", default_value=serial_baudrate),
        DeclareLaunchArgument("frame_id", default_value=frame_id),
        DeclareLaunchArgument("inverted", default_value=inverted),
        sllidar_node,
    ])
