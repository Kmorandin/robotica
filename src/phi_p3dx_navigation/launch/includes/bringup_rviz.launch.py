"""
Launch auxiliar para configurar RViz.

Este launch:
- Lança RViz com configuração padrão para visualização do robô.

Argumentos:
- rviz_config: Caminho para config RViz (padrão: rviz.rviz).
- use_sim_time: Usar tempo de simulação (padrão: false).
- robot_namespace: Namespace para o robô.

Incluído por: bringup_gazebo.launch.py, bringup_mobilesim.launch.py
"""

from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

## configura spawing de rviz
def generate_launch_description():
    pkg_description = get_package_share_directory("phi_p3dx_description")

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=join(pkg_description, "rviz", "rviz.rviz"),
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false"
    )

    namespace_arg = DeclareLaunchArgument(
        "robot_namespace",
        default_value="",
        description="pioneer3dx"
    )

    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("robot_namespace")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            {"use_sim_time": use_sim_time}
        ],
    )

    return LaunchDescription([
        rviz_config_arg,
        use_sim_time_arg,
        namespace_arg,
        rviz_node,
    ])