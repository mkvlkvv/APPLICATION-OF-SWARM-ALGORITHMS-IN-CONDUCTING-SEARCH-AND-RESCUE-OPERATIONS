from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    drone_id = LaunchConfiguration('drone_id')
    altitude = LaunchConfiguration('altitude')
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='1'),
        DeclareLaunchArgument('altitude', default_value='8.0'),
        Node(
            package='swarm_explorer',
            executable='explorer',
            name=['explorer_', drone_id],
            output='screen',
            parameters=[{
                'drone_id': drone_id,
                'altitude': altitude,
            }],
        ),
    ])