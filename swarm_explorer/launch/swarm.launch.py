from launch import LaunchDescription
from launch_ros.actions import Node

# (drone_id, altitude)
DRONES = [
    (1, 7.0),
    (2, 8.0),
    (3, 9.0),
    (4, 10.0),
    (5, 11.0),
]


def generate_launch_description():
    nodes = []
    for did, alt in DRONES:
        nodes.append(Node(
            package="swarm_explorer",
            executable="explorer",
            name=f"explorer_{did}",
            output="screen",
            parameters=[{
                "drone_id": did,
                "altitude": alt,
                "arena_xmin": -24.0,
                "arena_xmax":  24.0,
                "arena_ymin": -24.0,
                "arena_ymax":  24.0,
                "tick_dt": 1.0,
                "reach_r": 1.5,
                "recruit_radius": 15.0,
                "recruit_threshold": 0.5,
                "self_target_radius": 6.0,
                "known_target_radius": 5.0,
                "inspect_duration": 4.0,
                "levy_xmin": 3.0,
                "levy_xmax": 12.0,
                "levy_alpha": 1.5,
                "gradient_weight": 0.75,
            }],
        ))
    return LaunchDescription(nodes)