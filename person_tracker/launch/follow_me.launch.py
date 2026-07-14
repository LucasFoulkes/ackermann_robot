"""Second command of the follow-me flow.

  1. bash .../tools/preflight_session.sh        (armed stack, READY)
  2. ros2 launch person_tracker follow_me.launch.py

Starts the leg tracker and the follower. Walk a meter so the tracker
confirms you (green marker), then the robot engages and holds a ~0.9 m
standoff. Stand still and it stops with you; disappear for 3 s and it
cancels; live speed: tools/set_speed.sh <m/s>.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='person_tracker', executable='person_tracker',
             output='screen'),
        Node(package='person_tracker', executable='person_follower',
             output='screen'),
    ])
