import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ackermann_robot_dir = get_package_share_directory('ackermann_robot')
    
    # 1. Bringup (Sensors, Odom, Driver, Bridge, SLAM)
    # This launches:
    # - ldlidar_stl_ros2 (LiDAR)
    # - rf2o_laser_odometry (Odomg)
    # - slam_toolbox (SLAM / Map)
    # - ackermann_driver (Motors)
    # - cmd_vel_bridge (Twist -> Ackermann)
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ackermann_robot_dir, 'launch', 'bringup.launch.py')
        )
    )

    params_file = os.path.join(ackermann_robot_dir, 'config', 'nav2_params.yaml')

    # 2. Navigation (Nav2)
    # This launches:
    # - planner_server (SmacHybrid)
    # - controller_server (RegulatedPurePursuit)
    # - bt_navigator
    # - recoveries_server
    # - costmaps (local & global)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ackermann_robot_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'params_file': params_file}.items()
    )

    return LaunchDescription([
        bringup_launch,
        navigation_launch
    ])
