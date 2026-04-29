from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'swarm_pheromone'

setup(
    name=package_name,
    version='0.0.1',
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
    maintainer='mkvlkv',
    maintainer_email='you@example.com',
    description='Pheromone map server for swarm coverage',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pheromone_server = swarm_pheromone.pheromone_server:main',
        ],
    },
)
