import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    ackermann_robot_dir = get_package_share_directory('ackermann_robot')
    
    # 1. SLAM + Navigation
    # This reuses the existing autonomy.launch.py which sets up:
    # - Hardware (LiDAR, Motors)
    # - Odometry (RF2O)
    # - SLAM (Slam Toolbox)
    # - Navigation (Nav2 with optimized params)
    slam_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ackermann_robot_dir, 'launch', 'autonomy.launch.py')
        )
    )

    # 2. Explore Lite
    # This node autonomously drives the robot to unknown areas
    explore_config = os.path.join(
        ackermann_robot_dir,
        'config',
        'explore_params.yaml'
    )

    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore',
        output='screen',
        parameters=[explore_config]
    )

    return LaunchDescription([
        slam_nav_launch,
        explore_node
    ])
