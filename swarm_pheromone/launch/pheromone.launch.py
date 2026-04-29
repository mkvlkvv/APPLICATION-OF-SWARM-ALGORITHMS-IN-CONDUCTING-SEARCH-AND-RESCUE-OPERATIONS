from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='swarm_pheromone',
            executable='pheromone_server',
            name='pheromone_server',
            output='screen',
            parameters=[{
                'origin_x': -25.0,
                'origin_y': -25.0,
                'cell_size': 1.0,
                'width': 50,
                'height': 50,
                'tau_explored': 60.0,
                'tau_target': 300.0,
                'num_agents': 5,
                'agent_origins_x': [0.0, 0.0, 0.0, 0.0, 0.0],
                'agent_origins_y': [0.0, 3.0, -3.0, 6.0, -6.0],
            }],
        ),
    ])