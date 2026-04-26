"""
Launch auxiliar para publishers de estado do robô.

Este launch:
- Gera a descrição do robô (URDF/XACRO).
- Lança robot_state_publisher para publicar TF.
- Lança joint_state_publisher para estados das juntas.

Argumentos:
- robot_namespace: Namespace para o robô.
- use_sim_time: Usar tempo de simulação.

Incluído por: bringup_spawn_gz.launch.py, bringup_rviz.launch.py (se necessário)
"""

from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_description = get_package_share_directory("phi_p3dx_description")

    namespace = LaunchConfiguration("robot_namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    urdf_file = join(pkg_description, "urdf", "p3dx", "pioneer3dx.xacro")

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_namespace", default_value="", description="Namespace do robô"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Usar tempo de simulação"),
        robot_state_publisher,
        joint_state_publisher,
    ])