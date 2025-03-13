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
    
    # Configurar IGNITION_MODEL_PATH
    if 'IGNITION_MODEL_PATH' in os.environ:
        ignition_model_path = os.environ['IGNITION_MODEL_PATH'] + os.pathsep + models_dir
    else:
        ignition_model_path = models_dir

    set_ignition_model_path_cmd = SetEnvironmentVariable("IGNITION_MODEL_PATH", ignition_model_path)

    # Configurar IGN_GAZEBO_RESOURCE_PATH para que Ignition encuentre los modelos y recursos
    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        ign_gazebo_resource_path = os.environ['IGN_GAZEBO_RESOURCE_PATH'] + os.pathsep + gps_wpf_dir
    else:
        ign_gazebo_resource_path = gps_wpf_dir

    set_ign_gazebo_resource_path_cmd = SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", ign_gazebo_resource_path)
    
    # Iniciar Ignition Gazebo con el mundo especificado
    start_ignition_server_cmd = ExecuteProcess(
        cmd=['ign', 'gazebo', world, '-r'],
        output='both',
        shell=True
    )
    
    # Incluir la descripción del puente ROS-Gazebo (si es necesario)
    bridge_py_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bridge.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()  # Habilitar uso del tiempo simulado
    )

    # Incluir la descripción del robot_state_publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'local_robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Ruta del archivo de descripción del robot
    robot_description_file = os.path.join(gps_wpf_dir, 'models/turtlebot_waffle_gps', 'model_ign.sdf')

    # Comando para generar el robot en el mundo
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot3',
            '-x', '-137',
            '-z', '5.5',
            '-y', '-21',
            '-file', robot_description_file
        ],
        output='screen'
    )

    # Nodo para la publicación del estado de las articulaciones
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Crear la descripción del lanzamiento
    ld = LaunchDescription()
    ld.add_action(set_ignition_model_path_cmd)
    ld.add_action(set_ign_gazebo_resource_path_cmd)
    ld.add_action(start_ignition_server_cmd)
    ld.add_action(bridge_py_cmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn)
    ld.add_action(joint_state_publisher_node)

    return ld
