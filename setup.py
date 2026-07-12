from glob import glob
from os.path import join

from setuptools import find_packages, setup

package_name = 'ackermann_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (join('share', package_name, 'docs'), glob('docs/*.md')),
        (join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (join('share', package_name, 'config'), glob('config/*')),
        (join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luky',
    maintainer_email='luky@todo.todo',
    description='Clean starting point for the Ackermann robot.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'adaptive_ackermann_controller = ackermann_robot.adaptive_ackermann_controller:main',
            'path_segment_dispatcher = ackermann_robot.path_segment_dispatcher:main',
            'esc_forward_calibrate = ackermann_robot.experiments.esc_forward_calibrate:main',
            'esc_system_id = ackermann_robot.experiments.esc_system_id:main',
            'esc_speed_control_test = ackermann_robot.experiments.esc_speed_control_test:main',
            'static_odom_noise = ackermann_robot.experiments.static_odom_noise:main',
            'analyze_odom_observer = ackermann_robot.experiments.analyze_odom_observer:main',
            'analyze_preview_replay = ackermann_robot.experiments.analyze_preview_replay:main',
            'steering_sanity = ackermann_robot.experiments.steering_sanity:main',
            'steering_system_id = ackermann_robot.experiments.steering_system_id:main',
            'steering_staircase_id = ackermann_robot.experiments.steering_staircase_id:main',
            'steering_step_id = ackermann_robot.experiments.steering_step_id:main',
            'esc_sustain_floor_id = ackermann_robot.experiments.esc_sustain_floor_id:main',
            'esc_direction_flip_id = ackermann_robot.experiments.esc_direction_flip_id:main',
            'steering_limit_search = ackermann_robot.experiments.steering_limit_search:main',
        ],
    },
)
