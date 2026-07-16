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
            'tf_odom_bridge = ackermann_robot.tf_odom_bridge:main',
        ],
    },
)
