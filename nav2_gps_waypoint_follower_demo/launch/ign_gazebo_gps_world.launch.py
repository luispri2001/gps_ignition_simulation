import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Obtener el directorio del paquete
    gps_wpf_dir = get_package_share_directory("nav2_gps_waypoint_follower_demo")
    launch_dir = os.path.join(gps_wpf_dir, 'launch')
    world = os.path.join(gps_wpf_dir, "worlds", "n1_ign.world")

    # Agregar los directorios de modelos
    models_dir = os.path.join(gps_wpf_dir, "models")
    models_dir += os.pathsep + f"/opt/ros/{os.getenv('ROS_DISTRO')}/share/turtlebot3_gazebo/models"
    
    # Establecer la variable de entorno para Ignition
    if 'IGNITION_MODEL_PATH' in os.environ:
        ignition_model_path = os.environ['IGNITION_MODEL_PATH'] + os.pathsep + models_dir
    else:
        ignition_model_path = models_dir
    set_ignition_model_path_cmd = SetEnvironmentVariable("IGNITION_MODEL_PATH", ignition_model_path)
    
    # Iniciar Ignition Gazebo con el mundo especificado
    start_ignition_server_cmd = ExecuteProcess(
        cmd=['ign', 'gazebo', world, '-r'],
        output='both',
        shell=True
    )
    
    # Comando para incluir la descripción del puente (puede ser necesario dependiendo del sistema)
    bridge_py_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bridge.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()  # Habilitar uso del tiempo simulado
    )
    # Comando para incluir la descripción del robot_state_publisher (puede ser necesario dependiendo del sistema)
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'local_robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()  # Habilitar uso del tiempo simulado
    )


    # Crear el nodo joint_state_publisher con el formato adecuado
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]  # Usa el tiempo simulado de ROS
    )

    # Crear el LaunchDescription e incluir las acciones
    ld = LaunchDescription()
    ld.add_action(set_ignition_model_path_cmd)
    ld.add_action(start_ignition_server_cmd)  # Asegúrate de que se ejecute inmediatamente
    ld.add_action(bridge_py_cmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_node)  # Agregar el nodo de joint_state_publisher
    return ld
