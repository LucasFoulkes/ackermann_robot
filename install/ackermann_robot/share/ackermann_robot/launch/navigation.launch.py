import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ackermann_share_dir = get_package_share_directory("ackermann_robot")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    
    params_file = os.path.join(ackermann_share_dir, "config", "nav2_params.yaml")
    
    # We only need the navigation nodes, not localization (SLAM is doing that)
    # We can use the nav2_bringup navigation_launch.py
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": params_file,
            "autostart": "true",
        }.items(),
    )

    return LaunchDescription([
        nav2_launch,
    ])
