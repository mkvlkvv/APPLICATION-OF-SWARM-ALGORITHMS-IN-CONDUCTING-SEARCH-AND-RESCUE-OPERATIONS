from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'swarm_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maria Kovalkova',
    maintainer_email='mkvlkv@example.com',
    description='Swarm offboard controller for PX4 SITL via ROS 2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'offboard_agent = swarm_controller.offboard_agent:main',
        ],
    },
)