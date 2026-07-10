#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
        parameters=[{
            'robot_description': ParameterValue(
                robot_description,
                value_type=str,
            ),
        }],
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

    mola_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mola_lidar_odometry'),
                'ros2-launchs',
                'ros2-lidar-odometry.launch.py',
            ])
        ),
        launch_arguments={
            # Native C1 LaserScan input; no PointCloud conversion.
            'lidar_topic_name': '/scan',
            'lidar_topic_type': 'LaserScan',
            'lidar_qos_reliability': 'best_effort',
            'lidar_qos_depth': '20',
            'lidar_scan_validity_minimum_point_count': '100',
            # Use the measured base_link -> laser transform from the Xacro.
            'mola_tf_base_link': 'base_link',
            'ignore_lidar_pose_from_tf': 'False',
            # Native point-to-grid-map 2D ICP pipeline.
            'mola_lo_pipeline': '../pipelines/lidar2d.yaml',
            'enforce_planar_motion': 'True',
            'mola_deskew_method': 'MotionCompensationMethod::Linear',
            # MOLA is our odometry source: publish odom -> base_link directly.
            'mola_lo_reference_frame': 'odom',
            'publish_localization_following_rep105': 'False',
            'use_state_estimator': 'False',
            # Build the rolling local map needed by scan-to-map ICP.
            'start_active': 'True',
            'start_mapping_enabled': 'True',
            'generate_simplemap': 'False',
            # Headless robot: never launch local visualization processes.
            'use_mola_gui': 'False',
            'use_rviz': 'False',
            'use_diagnostic_aggregator': 'False',
            'use_sim_time': 'False',
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
        mola_odometry,
    ])
