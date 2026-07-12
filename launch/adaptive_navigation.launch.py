"""Odom-local Nav2 experiment with adaptive Ackermann actuator control."""
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ackermann_robot.adaptive_model import (
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
    prior_radius = float(parameters['planner_trackability_prior_radius_m'])
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
    (curvature_limit, controller_radius, planner_radius,
     trackability_path, utilization, prior_radius,
     planner_source, planner_confidence) = (
        learned_navigation_envelope(control))
    actuator_envelope = {'maximum_curvature_1pm': curvature_limit}
    planner_envelope = {
        'GridBased.minimum_turning_radius': planner_radius,
    }
    controller_envelope = {
        'FollowPath.regulated_linear_scaling_min_radius': planner_radius,
    }
    nodes = [
        Node(package='ackermann_robot', executable='adaptive_ackermann_controller',
             output='screen', parameters=[control, actuator_envelope, {
                 'arm_hardware': ParameterValue(arm, value_type=bool)}]),
        Node(package='ackermann_robot', executable='path_segment_dispatcher',
             output='screen', parameters=[{
                 'frontend_action_name': 'follow_path',
                 'backend_action_name': 'follow_path_backend',
                 'cusp_settle_time_s': 0.60,
                 'cusp_goal_checker_id': 'cusp_goal_checker',
                 'minimum_executable_segment_m': 0.18,
                 'segment_no_progress_timeout_s': 6.0,
                 'segment_progress_epsilon_m': 0.05,
                 'segment_wrong_direction_timeout_s': 0.75,
                 'segment_watch_min_length_m': 0.30,
                 'segment_path_check_period_s': 0.50,
                 'segment_path_check_horizon_m': 1.50,
                 'segment_blocked_path_timeout_s': 0.75,
                 'trackability_state_path': trackability_path,
                 'trackability_prior_radius_m': prior_radius,
                 'trackability_physical_limit_1pm': (
                     curvature_limit * utilization)}]),
        Node(package='nav2_planner', executable='planner_server', output='screen',
             parameters=[nav, planner_envelope]),
        Node(package='nav2_controller', executable='controller_server', output='screen',
             parameters=[nav, controller_envelope],
             remappings=[('cmd_vel', 'cmd_vel_nav_raw'),
                         ('follow_path', 'follow_path_backend')]),
        Node(package='nav2_behaviors', executable='behavior_server', output='screen',
             parameters=[nav], remappings=[('cmd_vel', 'cmd_vel_nav_raw')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', output='screen',
             parameters=[nav, {'default_nav_to_pose_bt_xml': tree}]),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             output='screen', parameters=[nav]),
    ]
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'autostart': True, 'bond_timeout': 0.0,
                     'node_names': ['planner_server', 'controller_server',
                                    'behavior_server', 'bt_navigator',
                                    'collision_monitor']}])
    return LaunchDescription([
        DeclareLaunchArgument('arm_hardware', default_value='false',
                              description='Actually drive PCA9685 outputs'),
        LogInfo(msg=(
            'Learned Ackermann envelope: '
            f'physical curvature={curvature_limit:.3f} 1/m, '
            f'physical radius={controller_radius:.3f} m, '
            f'planned radius={planner_radius:.3f} m '
            f'({planner_source}, confidence={planner_confidence:.2f})')),
        *nodes, lifecycle])
