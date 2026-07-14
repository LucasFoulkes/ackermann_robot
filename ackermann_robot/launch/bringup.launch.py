#!/usr/bin/env python3

import os
import time

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# Session hygiene, embedded so ONE `ros2 launch` is the whole entry point
# (replaces the preflight shell wrapper): kill any previous session's nodes
# and stale `ros2 launch`/bag parents (never our own), clear stale FastDDS
# shared-memory locks (hard-kill leftovers silently break discovery), and
# probe the PCA9685 so a clamped I2C bus is announced up front.
CLEANUP = r"""
for pid in $(pgrep -f '[c]omponent_container|[c]ontroller_server|[p]lanner_server|[b]ehavior_server|[s]moother_server|[b]t_navigator|[l]ifecycle_manager|[c]ollision_monitor|[v]elocity_smoother|[p]ath_segment_dispatcher|[a]daptive_ackermann_controller|[p]ca9685_effort_driver|[t]f_odom_bridge|[p]erson_tracker|[p]erson_follower|[r]obot_state_publisher|[m]ola|[s]llidar|[r]os2 bag'); do
  kill -9 $pid 2>/dev/null
done
for pid in $(pgrep -f '[r]os2 launch'); do
  [ "$pid" != "$PPID" ] && kill -9 $pid 2>/dev/null
done
sleep 1
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
if ! timeout 8 i2cget -y 1 0x40 0x00 >/dev/null 2>&1; then
  echo 'WARNING: PCA9685 not answering at 0x40 (HAT unpowered, or bus clamped -> full Pi power-off needed)'
fi
echo 'session cleanup done'
"""


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    arm_hardware = LaunchConfiguration('arm_hardware')
    record_bag = LaunchConfiguration('record_bag')
    follow = LaunchConfiguration('follow')

    bag_directory = os.path.expanduser('~/.robot/bags')
    os.makedirs(bag_directory, exist_ok=True)
    bag_path = os.path.join(
        bag_directory, time.strftime('adaptive_drive_%Y%m%d_%H%M%S'))

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

    # The navigation stack starts immediately but remains safe while lidar/MOLA
    # initialize: stale scan or odometry forces the controller to command
    # neutral effort, and the driver's reference-timeout deadman holds neutral
    # whenever the controller itself goes quiet.
    adaptive_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ackermann_robot'),
                'launch',
                'adaptive_navigation.launch.py',
            ])
        ),
        launch_arguments={
            'arm_hardware': arm_hardware,
        }.items(),
    )

    # follow:=true adds person detection + direct-pursuit following on
    # top of the normal stack (one-command follow-me session).
    person_tracker = Node(
        package='person_tracker', executable='person_tracker',
        output='screen', condition=IfCondition(follow))
    person_follower = Node(
        package='person_tracker', executable='person_follower',
        output='screen', condition=IfCondition(follow))

    flight_recorder = ExecuteProcess(
        condition=IfCondition(record_bag),
        cmd=[
            'ros2', 'bag', 'record',
            '/tf', '/tf_static', '/scan', '/odom',
            '/cmd_vel_nav_raw', '/cmd_vel_nav', '/cmd_vel',
            '/actuator_effort', '/driver/debug', '/speed_limit',
            '/person_tracker/person', '/person_tracker/people',
            '/plan', '/unsmoothed_plan', '/controller_segment_plan',
            '/controller/debug', '/controller/limits',
            '/planner_trackability',
            '/collision_monitor_state',
            '/local_costmap/costmap', '/global_costmap/costmap',
            '-o', bag_path,
        ],
        output='screen',
    )

    cleanup = ExecuteProcess(cmd=['bash', '-c', CLEANUP],
                             name='session_cleanup', output='screen')
    stack = [
        robot_state_publisher,
        lidar,
        mola_odometry,
        adaptive_navigation,
        person_tracker,
        person_follower,
        flight_recorder,
    ]
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
        DeclareLaunchArgument(
            'arm_hardware',
            default_value='false',
            description=(
                'Arm the pca9685_effort_driver (physical throttle and '
                'steering output). False runs the complete stack, including '
                'the driver, with no I2C writes.'
            ),
        ),
        DeclareLaunchArgument(
            'record_bag',
            default_value='true',
            description=(
                'Record synchronized navigation and controller topics under '
                '~/.robot/bags (enabled by default).'
            ),
        ),
        DeclareLaunchArgument(
            'follow',
            default_value='false',
            description='Also start person_tracker + person_follower '
                        '(follow-me mode).',
        ),
        cleanup,
        RegisterEventHandler(OnProcessExit(
            target_action=cleanup, on_exit=stack)),
    ])
