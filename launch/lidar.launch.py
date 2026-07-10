#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')

    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('ackermann_robot'),
            'urdf',
            'ackermann_robot.urdf.xacro',
        ]),
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('sllidar_ros2'),
                'launch',
                'sllidar_c1_launch.py',
            ])
        ),
        launch_arguments={
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': 'laser',
            'inverted': 'false',
            'angle_compensate': 'true',
            'scan_mode': 'Standard',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial device connected to the RPLIDAR C1.',
        ),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='460800',
            description='RPLIDAR C1 serial baud rate.',
        ),
        robot_state_publisher,
        lidar,
    ])
