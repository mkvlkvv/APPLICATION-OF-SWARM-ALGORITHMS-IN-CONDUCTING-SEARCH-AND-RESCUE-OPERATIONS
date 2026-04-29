from setuptools import find_packages, setup

package_name = 'swarm_metrics'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mkvlkv',
    maintainer_email='mkvlkv@example.com',
    description='Swarm metrics logger (targets, coverage, path length).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'metrics = swarm_metrics.metrics:main',
        ],
    },
)
