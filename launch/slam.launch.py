#!/usr/bin/env python3
"""slam_toolbox online async mapping — builds the map live and publishes map->odom.

Consumes /scan + the odom->base_link TF (from rf2o) and provides the map frame +
the map->odom correction TF that Nav2 needs.

slam_toolbox nodes are LIFECYCLE nodes on Jazzy and do NOT self-activate from a
plain Node action — they sit in 'unconfigured' and never subscribe to /scan. So
we include slam_toolbox's own online_async_launch.py, which wires the
configure->activate lifecycle transitions (autostart:=true), and just feed it our
params file. Run alongside the lidar, URDF TF, and rf2o.

Save the map once built:
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_map'}}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory("ackermann_robot")
    default_slam_params = os.path.join(pkg, "config", "slam_toolbox_online_async.yaml")
    slam_params = LaunchConfiguration("slam_params_file", default=default_slam_params)
    slam_launch = os.path.join(
        get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py"
    )
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            "slam_params_file": slam_params,
            "use_sim_time": use_sim_time,
            "autostart": "true",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value=use_sim_time),
        DeclareLaunchArgument("slam_params_file", default_value=default_slam_params),
        slam,
    ])
