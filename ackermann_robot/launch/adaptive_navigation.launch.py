"""Odom-local Nav2 experiment with adaptive Ackermann actuator control."""
import math
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from adaptive_ackermann.adaptive_model import (
    conservative_curvature_limit, learned_planner_curvature,
    TrackabilityEstimator)


def learned_navigation_envelope(control_path):
    """Return physical curvature plus trackable planner/controller radii."""
    with open(control_path) as stream:
        control = yaml.safe_load(stream) or {}
    parameters = control['adaptive_ackermann_controller']['ros__parameters']
    nominal = float(parameters['maximum_curvature_1pm'])
    runtime_path = os.path.expanduser(parameters['runtime_model_path'])
    runtime = {}
    try:
        with open(runtime_path) as stream:
            runtime = yaml.safe_load(stream) or {}
    except (OSError, yaml.YAMLError):
        pass
    curvature = conservative_curvature_limit(
        nominal, runtime.get('steering_rls_models', {}),
        minimum_observations=int(parameters['steering_rls_full_confidence_observations']),
        residual_limit=float(parameters['steering_rls_residual_limit_1pm']))
    utilization = float(parameters['planning_curvature_utilization'])
    trackability_path = os.path.expanduser(
        parameters['planner_trackability_state_path'])
    trackability = {}
    try:
        with open(trackability_path) as stream:
            trackability = yaml.safe_load(stream) or {}
    except (OSError, yaml.YAMLError):
        pass
    # Geometry-scaled bootstrap (doc §11.1): the planner prior derives from
    # the birth-certificate wheelbase and a documented cautious wheel angle,
    # not a universal meter value. A configured prior_radius <= 0 selects the
    # geometric derivation; a positive value remains an explicit override.
    prior_radius = float(parameters['planner_trackability_prior_radius_m'])
    if prior_radius <= 0.0:
        wheelbase = float(parameters['vehicle_wheelbase_m'])
        cautious_angle = math.radians(float(parameters.get(
            'planner_bootstrap_wheel_angle_deg', 12.0)))
        prior_radius = wheelbase / math.tan(cautious_angle)
    planning_curvature = learned_planner_curvature(
        curvature, utilization,
        prior_radius, trackability)
    estimator = TrackabilityEstimator(
        min(curvature * utilization, 1.0 / prior_radius),
        curvature * utilization, state=trackability)
    return (curvature, 1.0 / curvature, 1.0 / planning_curvature,
            trackability_path, utilization, prior_radius,
            estimator.source, estimator.confidence)


def generate_launch_description():
    share = get_package_share_directory('ackermann_robot')
    nav = os.path.join(share, 'config', 'nav2_odom_experiment.yaml')
    control = os.path.join(share, 'config', 'adaptive_controller.yaml')
    tree = os.path.join(share, 'config', 'navigate_to_pose_ackermann.xml')
    arm = LaunchConfiguration('arm_hardware')
    record = LaunchConfiguration('record_telemetry')
    (curvature_limit, controller_radius, planner_radius,
     trackability_path, utilization, prior_radius,
     planner_source, planner_confidence) = (
        learned_navigation_envelope(control))
    actuator_envelope = {'maximum_curvature_1pm': curvature_limit}
    planner_envelope = {
        'GridBased.minimum_turning_radius': planner_radius,
    }
    # NOTE: regulated_linear_scaling_min_radius deliberately NOT tied to
    # the planner radius anymore. Coupling them meant 'can plan tighter'
    # silently became 'corners faster': when the learned planning radius
    # shrank 1.33 -> 0.85 m, tight-curve speeds doubled (0.17 -> 0.33 m/s
    # median) and turning quality collapsed (2026-07-14). Cornering speed
    # is a tracking-comfort limit and stays fixed in the yaml.
    controller_envelope = {}
    # Installed with the package (standard); rebuild after editing the
    # vehicle facts (colcon build --packages-select ackermann_robot).
    birth_certificate = os.path.join(
        share, 'config', 'birth_certificate.yaml')
    nodes = [
        # Hardware split (2026-07-13): the controller is hardware-free and
        # publishes normalized efforts; the driver owns the PCA9685 (arming,
        # deadman, neutral-on-error) using the birth-certificate endpoints.
        Node(package='adaptive_ackermann',
             executable='adaptive_ackermann_controller',
             output='screen', parameters=[control, actuator_envelope, {
                 'birth_certificate_path': birth_certificate,
                 'record_telemetry': ParameterValue(
                     record, value_type=bool)}]),
        Node(package='pca9685_effort_driver',
             executable='pca9685_effort_driver',
             output='screen', parameters=[{
                 'arm_hardware': ParameterValue(arm, value_type=bool),
                 'birth_certificate_path': birth_certificate,
                 'reference_timeout_s': 0.5}]),
        # MOLA publishes TF only; this bridge turns odom->base_link into the
        # /odom topic every consumer (controller, dispatcher, Nav2) reads.
        Node(package='ackermann_robot', executable='tf_odom_bridge',
             output='screen'),
        Node(package='adaptive_ackermann', executable='path_segment_dispatcher',
             output='screen', parameters=[{
                 'frontend_action_name': 'follow_path',
                 'backend_action_name': 'follow_path_backend',
                 # 0.30 was SERIAL with the controller's 0.5 s direction debounce —
                 # two dwells guarding the same reversal. The debounce alone
                 # is the certified guard; a token settle only covers the
                 # backend-goal round-trip (flip decomposition 2026-07-15:
                 # 2.2 s = decel 0.5 + handoff 0.8 + debounce 0.5 + launch 0.4).
                 'cusp_settle_time_s': 0.10,
                 'cusp_goal_checker_id': 'cusp_goal_checker',
                 # 0.18 let Smac's 2-3-pose alignment nubs execute as
                 # full stop-flip-step cycles (27% of movement episodes
                 # were <1.5 s 'weird steps', 01:22 run). 0.30: nubs get
                 # skipped and their neighbors MERGED; the capped
                 # steering feedback absorbs the ~0.2 m kink while
                 # rolling. Real tight-quarter cusp legs are >=0.35 m at
                 # the 0.95 m planning radius.
                 'minimum_executable_segment_m': 0.30,
                 'segment_no_progress_timeout_s': 6.0,
                 'segment_progress_epsilon_m': 0.05,
                 'segment_wrong_direction_timeout_s': 0.75,
                 # Every segment we permit the dispatcher to execute is also
                 # protected by the chronological progress watchdog.
                 'segment_watch_min_length_m': 0.18,
                 'segment_path_check_period_s': 0.50,
                 'segment_path_check_horizon_m': 1.50,
                 'segment_blocked_path_timeout_s': 0.75,
                 'trackability_state_path': trackability_path,
                 'trackability_prior_radius_m': prior_radius,
                 'trackability_physical_limit_1pm': (
                     curvature_limit * utilization)}]),
        # respawn + bonds (2026-07-16 12:58: collision_monitor segfaulted
        # mid-session, cutting the cmd_vel chain — the robot was bricked
        # until relaunch). A respawned lifecycle node is useless without
        # the manager re-activating it, hence bonds back on: the manager
        # notices the death and reconnects within bond_respawn_max.
        Node(package='nav2_planner', executable='planner_server', output='screen',
             respawn=True, respawn_delay=2.0,
             parameters=[nav, planner_envelope]),
        Node(package='nav2_controller', executable='controller_server', output='screen',
             respawn=True, respawn_delay=2.0,
             parameters=[nav, controller_envelope],
             remappings=[('cmd_vel', 'cmd_vel_nav_raw'),
                         ('follow_path', 'follow_path_backend')]),
        Node(package='nav2_behaviors', executable='behavior_server', output='screen',
             respawn=True, respawn_delay=2.0,
             parameters=[nav], remappings=[('cmd_vel', 'cmd_vel_nav_raw')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', output='screen',
             respawn=True, respawn_delay=2.0,
             parameters=[nav, {'default_nav_to_pose_bt_xml': tree}]),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             output='screen', respawn=True, respawn_delay=2.0,
             parameters=[nav]),
    ]
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'autostart': True, 'bond_timeout': 4.0,
                     'attempt_respawn_reconnection': True,
                     'bond_respawn_max_duration': 10.0,
                     'node_names': ['planner_server', 'controller_server',
                                    'behavior_server', 'bt_navigator',
                                    'collision_monitor']}])
    return LaunchDescription([
        DeclareLaunchArgument('record_telemetry', default_value='false',
                              description='Write the flight-recorder CSV '
                              '(diagnostic only; learning is online)'),
        DeclareLaunchArgument('arm_hardware', default_value='false',
                              description='Actually drive PCA9685 outputs'),
        LogInfo(msg=(
            'Learned Ackermann envelope: '
            f'physical curvature={curvature_limit:.3f} 1/m, '
            f'physical radius={controller_radius:.3f} m, '
            f'planned radius={planner_radius:.3f} m '
            f'({planner_source}, confidence={planner_confidence:.2f})')),
        *nodes, lifecycle])
