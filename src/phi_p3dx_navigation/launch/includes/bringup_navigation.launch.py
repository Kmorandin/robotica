"""
Launch auxiliar para o nó de navegação.

Este launch:
- Lança o nó obstacle_avoidance para controle reativo de desvio de obstáculos.

Argumentos:
- robot_namespace: Namespace.
- use_sim_time: Usar tempo de simulação.

Incluído por: bringup_gazebo.launch.py, bringup_mobilesim.launch.py, bringup_robot.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

## configura spawing the controller a ser usado
def generate_launch_description():
    namespace = LaunchConfiguration('robot_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    controller_node = Node(
        package='phi_p3dx_navigation',
        executable='control_example_py',
        name='control_example_py',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        controller_node
    ])
