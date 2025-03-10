# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Obtener directorios de paquetes
    bringup_dir = get_package_share_directory('nav2_bringup')
    gps_wpf_dir = get_package_share_directory('nav2_gps_waypoint_follower_demo')

    # Definir rutas a archivos de configuración
    launch_dir = os.path.join(gps_wpf_dir, 'launch')
    params_dir = os.path.join(gps_wpf_dir, "config")
    nav2_params = os.path.join(params_dir, "nav2_no_map_params.yaml")
    map_file = os.path.join(params_dir, "route_map.yaml")  # Ruta del mapa personalizado

    # Reescribir YAML para parámetros de Nav2
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )

    # Configuración de LaunchConfigurations
    use_rviz = LaunchConfiguration('use_rviz')
    use_mapviz = LaunchConfiguration('use_mapviz')

    # Declarar argumentos de lanzamiento
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='False', description='Si debe iniciar RVIZ'
    )

    declare_use_mapviz_cmd = DeclareLaunchArgument(
        'use_mapviz', default_value='False', description='Si debe iniciar mapviz'
    )

    # Lanzar Gazebo con el mundo
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'ign_gazebo_gps_world.launch.py'))
    )

    # Lanzar robot localization con dual ekf + GPS
    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'ign_dual_ekf_navsat.launch.py'))
    )

    # Lanzar Nav2 con el mapa personalizado
    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": configured_params,
            "autostart": "True",
            "map": map_file,
        }.items(),
    )

    # Lanzar RViz si está habilitado
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", 'rviz_launch.py')),
        condition=IfCondition(use_rviz)
    )

    # Lanzar Mapviz si está habilitado
    mapviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'mapviz.launch.py')),
        condition=IfCondition(use_mapviz)
    )

    # Crear el LaunchDescription y agregar acciones
    ld = LaunchDescription()

    # Agregar lanzamientos a la descripción
    ld.add_action(gazebo_cmd)  # Iniciar Gazebo
    ld.add_action(robot_localization_cmd)  # Iniciar localización del robot
    ld.add_action(navigation2_cmd)  # Iniciar Nav2 con el mapa
    ld.add_action(declare_use_rviz_cmd)  # Argumento para habilitar RViz
    ld.add_action(rviz_cmd)  # Iniciar RViz si se habilita
    ld.add_action(declare_use_mapviz_cmd)  # Argumento para habilitar Mapviz
    ld.add_action(mapviz_cmd)  # Iniciar Mapviz si se habilita

    return ld
