"""
Launch principal para simulação 2D com MobileSim.

Este launch configura e inicia:
- O simulador MobileSim com um mapa (obstacles.map).
- O driver phi_p3dx_aria para conectar ao MobileSim via rede.
- RViz opcionalmente para visualização.
- COMENTADO - O nó de navegação (obstacle_avoidance) para controle do robô.

Argumentos:
- port: Endereço do MobileSim (padrão: 'localhost:8101').
- robot_namespace: Namespace para o robô (padrão: vazio).
- use_rviz: Se deve abrir RViz (padrão: true).

Exemplo de uso:
  ros2 launch phi_p3dx_navigation bringup_mobilesim.launch.py port:=192.168.1.100:8101
"""

from os.path import join
from launch.substitutions import PathJoinSubstitution
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

## configura spawing do ambiente 2d "mobilesim"
def generate_launch_description():
    pkg_description = get_package_share_directory('phi_p3dx_description')
    pkg_navigation = get_package_share_directory('phi_p3dx_navigation')


    port_arg = DeclareLaunchArgument(
        'port', default_value='localhost:8101',
        description='')

    map_arg = DeclareLaunchArgument(
        'map_name', default_value='obstacles.map',
        description='')

    namespace_arg = DeclareLaunchArgument(
        'robot_namespace', default_value='',
        description='pioneer3dx')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='')

    port      = LaunchConfiguration('port')
    namespace = LaunchConfiguration('robot_namespace')
    use_rviz  = LaunchConfiguration('use_rviz')
    map_name  = LaunchConfiguration('map_name')

    map_file = PathJoinSubstitution([pkg_description, 'map', map_name]) 

    state_publishers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_navigation, 'launch', 'includes', 'bringup_state_publishers.launch.py')
        ),
        launch_arguments={
            'robot_namespace': namespace,
            'use_sim_time': 'false'
        }.items()
    )

    mobilesim = ExecuteProcess(
        cmd=['MobileSim', '-m', map_file],
        output='screen',
    )

    phi_aria_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='phi_p3dx_aria',
                executable='phi_p3dx',
                name='phi_p3dx_aria',
                namespace=namespace,
                output='screen',
                parameters=[{
                    'port':                port,
                    'odom_frame':          'odom',
                    'base_link_frame':     'base_link',
                    'sonar_frame':         'base_link',
                    'laser_frame':         'lidar_link',
                    'publish_aria_lasers': True,
                }]
            ),
        ],
    )

    navigation = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    join(pkg_navigation, 'launch', 'includes', 'bringup_navigation.launch.py')
                ),
                launch_arguments={
                    'robot_namespace': namespace
                }.items()
            ),
        ],
    )
    

    rviz_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    join(pkg_navigation, 'launch', 'includes', 'bringup_rviz.launch.py')
                ),
                condition=IfCondition(use_rviz),
                launch_arguments={
                    'robot_namespace': namespace,
                    'use_sim_time': 'false',
                }.items(),
            ),
        ],
    )

    return LaunchDescription([
        port_arg,
        namespace_arg,
        use_rviz_arg,
        map_arg,

        state_publishers,
        mobilesim,
        phi_aria_node,
        rviz_launch,
        # navigation,
    ])
