from setuptools import find_packages, setup

package_name = 'person_tracker'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/follow_person.xml']),
        ('share/' + package_name + '/launch',
         ['launch/follow_me.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lukas',
    maintainer_email='lukas@cananvalle.com',
    description='Stage-1 person detection from 2D lidar legs.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'person_tracker = person_tracker.person_tracker_node:main',
            'person_follower = person_tracker.person_follower:main',
        ],
    },
)
