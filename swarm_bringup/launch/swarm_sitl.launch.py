"""Launch: Gazebo + N PX4 SITL + MicroXRCEAgent."""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    num_drones_arg = DeclareLaunchArgument('num_drones', default_value='5')
    num_drones = LaunchConfiguration('num_drones')

    bringup_dir = get_package_share_directory('swarm_bringup')
    # Скрипт лежит в src, а не в share — используем абсолютный путь через HOME
    sitl_script = os.path.expanduser('~/swarm_ws_v1/src/swarm_bringup/scripts/sitl_multiple_run.sh')
    world_path = os.path.expanduser('~/swarm_ws_v1/src/swarm_bringup/worlds/swarm_arena.world')

    # 1) Micro XRCE-DDS Agent
    uxrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
        name='micro_xrce_agent'
    )

    # 2) SITL + Gazebo (через наш bash-скрипт)
    sitl = ExecuteProcess(
        cmd=['bash', sitl_script, num_drones, world_path, 'iris_down_cam'],
        output='screen',
        name='sitl_multiple'
    )

    # Запускаем агент чуть раньше SITL
    sitl_delayed = TimerAction(period=2.0, actions=[sitl])

    return LaunchDescription([
        num_drones_arg,
        uxrce_agent,
        sitl_delayed,
    ])