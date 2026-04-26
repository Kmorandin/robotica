"""
Launch principal para simulação completa no Gazebo 3D.

Este launch configura e inicia:
- O ambiente Gazebo com um mundo SDF (ex.: empty_world ou obstacles).
- O robô Pioneer 3-DX spawnado no mundo, com publishers de estado (robot_state_publisher, joint_state_publisher).
- A ponte ROS-Gazebo para comunicação entre ROS e Gazebo.
- RViz opcionalmente para visualização (padrão: ligado).
- COMENTADO - O nó de navegação (obstacle_avoidance) para controle do robô.

Argumentos:
- world_name: Nome do mundo SDF a carregar (padrão: 'obstacles').
- robot_namespace: Namespace para o robô (padrão: vazio).
- x, y, yaw: Posição inicial do robô no mundo.
- use_rviz: Se deve abrir RViz (padrão: true).

Exemplo de uso:
  ros2 launch phi_p3dx_navigation bringup_gazebo.launch.py world_name:=empty_world use_rviz:=false
"""

from os.path import join
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

## o arquivo main para subir o robô no gazebo 3D
def generate_launch_description():
    pkg_navigation = get_package_share_directory("phi_p3dx_navigation")
    pkg_description = get_package_share_directory("phi_p3dx_description")

    world_name = LaunchConfiguration("world_name")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    yaw = LaunchConfiguration("yaw")
    namespace = LaunchConfiguration("robot_namespace")
    use_rviz = LaunchConfiguration("use_rviz")

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_navigation, "launch", "includes", "bringup_env.launch.py")),
        launch_arguments={'world_name': world_name}.items()
    )

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_navigation, "launch", "includes", "bringup_spawn_gz.launch.py")),
        launch_arguments={
            'x': x,
            'y': y,
            'yaw': yaw,
            'robot_namespace': namespace
        }.items()
    )

    state_publishers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_navigation, "launch", "includes", "bringup_state_publishers.launch.py")),
        launch_arguments={
            'robot_namespace': namespace,
            'use_sim_time': 'true'
        }.items()
    )

    rviz_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(join(pkg_navigation, "launch", "includes", "bringup_rviz.launch.py")),
                condition=IfCondition(use_rviz),
                launch_arguments={
                    'robot_namespace': namespace,
                    'use_sim_time': 'true'
                }.items()
            )
        ]
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_navigation, "launch", "includes", "bringup_navigation.launch.py")),
        launch_arguments={
            'robot_namespace': namespace,
            'use_sim_time': 'true'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument("world_name", default_value="obstacles"),
        DeclareLaunchArgument("robot_namespace", default_value=""),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.5"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        
        world_launch,
        state_publishers,
        robot_launch,
        rviz_launch,
        # navigation_launch
    ])