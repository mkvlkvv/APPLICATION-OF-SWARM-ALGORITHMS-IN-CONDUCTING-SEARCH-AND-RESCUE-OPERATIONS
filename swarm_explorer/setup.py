import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'swarm_explorer'

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
    description='Swarm explorer nodes (SEARCH/RECRUIT/INSPECT) with pheromone stigmergy',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'explorer = swarm_explorer.explorer:main',
        ],
    },
)
