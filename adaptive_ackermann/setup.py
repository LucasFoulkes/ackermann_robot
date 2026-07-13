from setuptools import find_packages, setup

package_name = 'adaptive_ackermann'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lukas',
    maintainer_email='lukas@cananvalle.com',
    description='Self-learning Ackermann controller for encoder-less '
                'vehicles; hardware-free, effort-native.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'adaptive_ackermann_controller = '
            'adaptive_ackermann.adaptive_ackermann_controller:main',
            'path_segment_dispatcher = '
            'adaptive_ackermann.path_segment_dispatcher:main',
            'drive_scorecard = adaptive_ackermann.tools.drive_scorecard:main',
            'steering_map_challenger = '
            'adaptive_ackermann.tools.steering_map_challenger:main',
        ],
    },
)
