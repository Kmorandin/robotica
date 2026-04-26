"""
Launch auxiliar para spawnar o robô no Gazebo.

Este launch:
- Gera a descrição do robô (URDF/XACRO).
- Spawna o robô no Gazebo via ros_gz_sim/create.
- Lança a ponte ros_gz_bridge para comunicação ROS-Gazebo.

Argumentos:
- x, y, yaw: Posição inicial.
- robot_namespace: Namespace.

Incluído por: bringup_gazebo.launch.py
"""

from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

## configura spawing do gazebo 3D
def generate_launch_description():
    pkg_navigation = get_package_share_directory("phi_p3dx_navigation")
    pkg_description = get_package_share_directory("phi_p3dx_description")
    
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    yaw = LaunchConfiguration("yaw")
    namespace = LaunchConfiguration("robot_namespace")
    
    bridge_config = join(pkg_navigation, 'config', 'model_bridge.yaml')

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([pkg_description, 'urdf', 'p3dx', 'pioneer3dx.xacro']),
        ' robot_namespace:=', namespace
    ])

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        namespace=namespace,
        arguments=[
            '-topic', 'robot_description',
            '-name', 'pioneer3dx',
            '-allow_renaming', 'true',
            '-x', x, 
            '-y', y, 
            '-Y', yaw, 
            '-z', '0.2'
        ]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("robot_namespace", default_value=""),
        
        spawn_entity,
        bridge
    ])