"""
Launch auxiliar para configurar o ambiente Gazebo.

Este launch:
- Define variáveis de ambiente (GZ_SIM_RESOURCE_PATH) para mundos e modelos.
- Lança o Gazebo com um mundo SDF especificado.

Argumentos:
- world_name: Nome do mundo SDF (padrão: 'empty_world').

Incluído por: bringup_gazebo.launch.py
"""

from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

## configura spawing do ambiente (variaveis e o mundo)
def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_description = get_package_share_directory("phi_p3dx_description")

    world_name = LaunchConfiguration("world_name")
    
    from ament_index_python.packages import get_package_prefix
    pkg_install_dir = get_package_prefix("phi_p3dx_description")

    set_res_path_worlds = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
        value=join(pkg_description, 'worlds')
    )
    set_res_path_pkg = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
        value=join(pkg_install_dir, 'share')
    )
    set_res_path_models = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
        value=join(pkg_description, 'models')
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r ", join(pkg_description, "worlds", ""), world_name, ".sdf"]
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("world_name", default_value="empty_world"),
        set_res_path_worlds,
        set_res_path_pkg,
        set_res_path_models,
        gz_sim
    ])