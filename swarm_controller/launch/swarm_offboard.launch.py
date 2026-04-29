"""Запуск N offboard-агентов, по одному на дрон."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    num_drones_arg = DeclareLaunchArgument(
        'num_drones', default_value='5',
        description='Количество дронов в рое')
    takeoff_alt_arg = DeclareLaunchArgument(
        'takeoff_altitude', default_value='3.0',
        description='Высота взлёта (м)')

    ld = LaunchDescription([num_drones_arg, takeoff_alt_arg])

    # Генерируем 5 нод статически — ROS 2 launch не умеет range() нативно,
    # но можно через OpaqueFunction. Здесь проще — жёстко 5.
    for i in range(1, 6):
        ld.add_action(Node(
            package='swarm_controller',
            executable='offboard_agent',
            name=f'offboard_agent_{i}',
            output='screen',
            parameters=[{
                'drone_id': i,
                'takeoff_altitude': LaunchConfiguration('takeoff_altitude'),
            }],
        ))
    return ld