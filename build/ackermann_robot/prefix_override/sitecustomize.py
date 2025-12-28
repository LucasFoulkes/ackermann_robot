import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/luky/ros2_ws/src/ackermann_robot/install/ackermann_robot'
