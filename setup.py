import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ackermann_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.xml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.pgm')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luky',
    maintainer_email='luky@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ackermann_driver = ackermann_robot.ackermann_driver:main',
            'ps4_teleop = ackermann_robot.ps4_teleop:main',
            'drive_logger = ackermann_robot.drive_logger:main',
            'cmd_vel_to_effort = ackermann_robot.cmd_vel_to_effort:main',
            'web_teleop = ackermann_robot.web_teleop:main',
            'depth_floor_scan = ackermann_robot.depth_floor_scan:main',
            'steer_calibration = ackermann_robot.steer_calibration:main',
            'imu_throttle = ackermann_robot.imu_throttle:main',
            'system_stats = ackermann_robot.system_stats:main',
            'scan_throttle = ackermann_robot.scan_throttle:main',
        ],
    },
)
