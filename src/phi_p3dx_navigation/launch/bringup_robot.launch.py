"""
Launch principal para controle do robô real Pioneer 3-DX.

Este launch configura e inicia:
- O driver phi_p3dx_aria para conectar ao robô via rede (porta serial/TCP).
- O nó de navegação (obstacle_avoidance) para controle do robô.

Argumentos:
- port: Endereço do robô (padrão: '192.168.1.11:10002').
- robot_namespace: Namespace para o robô (padrão: vazio).

Exemplo de uso:
  ros2 launch phi_p3dx_navigation bringup_robot.launch.py port:=192.168.1.100:10002
"""

from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

## configura spawing do robô real
def generate_launch_description():
    pkg_phi_aria = get_package_share_directory('phi_p3dx_aria')
    pkg_navigation = get_package_share_directory('phi_p3dx_navigation')

    port_arg = DeclareLaunchArgument(
        'port', default_value='192.168.1.11:10002',
        description='')

    namespace_arg = DeclareLaunchArgument(
        'robot_namespace', default_value='',
        description='pioneer3dx')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Abrir RViz2 automaticamente')

    port      = LaunchConfiguration('port')
    namespace = LaunchConfiguration('robot_namespace')
    use_rviz  = LaunchConfiguration('use_rviz')


    state_publishers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_navigation, "launch", "includes", "bringup_state_publishers.launch.py")),
        launch_arguments={
            'robot_namespace': namespace,
            'use_sim_time': 'false'
        }.items()
    )

    phi_aria_node = Node(
        package='phi_p3dx_aria',
        executable='phi_p3dx',
        name='phi_p3dx_aria',
        namespace=namespace,
        output='screen',
        parameters=[{
            'port':       port,
            'odom_frame':       'odom',
            'base_link_frame':  'base_link',
            'sonar_frame':      'base_link',
            'laser_frame':      'lidar_link',
            'publish_aria_lasers': True,
        }]
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

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_navigation, "launch", "includes", "bringup_navigation.launch.py")),
        launch_arguments={
            'robot_namespace': namespace
        }.items()
    )

    
    return LaunchDescription([
        port_arg,
        namespace_arg,
        use_rviz_arg,


        state_publishers,
        phi_aria_node,
        rviz_launch,
        # navigation_launch,
    ])
