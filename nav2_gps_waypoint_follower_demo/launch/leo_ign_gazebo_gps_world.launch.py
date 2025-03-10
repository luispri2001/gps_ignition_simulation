import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Obtener los directorios de los paquetes
    leorover_urdf_path = get_package_share_directory('leorover_description')
    pkg_leorover_gazebo = get_package_share_directory('leorover_gazebo')
    pkg_leorover_gazebo_bridge = get_package_share_directory('leorover_gazebo_bridge')
    gps_wpf_dir = get_package_share_directory("nav2_gps_waypoint_follower_demo")
    world = os.path.join(gps_wpf_dir, "worlds", "campusReducido.world")
    
    # Obtener la descripción del robot en formato URDF
    xacro_file = os.path.join(leorover_urdf_path, 'urdf', 'leo_sim.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}
    
    # Archivo SDF del robot para Ignition Gazebo
    robot_description_file = os.path.join(pkg_leorover_gazebo, 'robots', 'leorover.sdf')
    
    # # Comando para establecer la variable de entorno de modelos de Ignition
    # models_dir = os.path.join(gps_wpf_dir, "models")
    # models_dir += os.pathsep + f"/opt/ros/{os.getenv('ROS_DISTRO')}/share/turtlebot3_gazebo/models"
    # ignition_model_path = os.environ.get('IGNITION_MODEL_PATH', '') + os.pathsep + models_dir
    # set_ignition_model_path_cmd = SetEnvironmentVariable("IGNITION_MODEL_PATH", ignition_model_path)
    
    # Iniciar Ignition Gazebo con el mundo especificado
    start_ignition_server_cmd = ExecuteProcess(
        cmd=['ign', 'gazebo', world, '-r'],
        output='both',
        shell=True
    )
    
    # Lanzar el puente ROS-Gazebo
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_leorover_gazebo_bridge, 'launch', 'leorover_gazebo_bridge.launch.py')),
    )

    # Lanzar robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )
    
    # Publicadores de TF estáticos
    static_tf_nodes = [
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_footprint_publisher',
        #     arguments=['0', '0', '0', '0', '0', '0', '1', '/leorover/base_footprint', 'base_footprint']
        # ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rp_lidar_publisher',
            arguments=['0', '0', '0', '0', '0', '1', '0', 'rp_lidar_optical_frame', 'leorover/rp_lidar_frame/gpu_lidar']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'camera_optical_frame', 'leorover/internal_camera_optical_frame/internal_camera']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='realsense_camera_publisher',
            arguments=['0', '0', '0', '-0.5', '0.5', '-0.5', '0.5', 'realsense_camera_link', 'leorover/realsense_camera_optical_frame/realsense_d455']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_to_base_link_publisher',
            arguments=['0.0', '0.0', '1.0', '0.0', '0.0', '0.0', '1.0', 'base_link', 'gps_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_publisher',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '1.0', 'map', 'odom']
        )
    ]
    
    # Lanzar el joint_state_publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Lanzar el nodo que spawnea el robot en Ignition Gazebo
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'leorover',
            '-x', '-169',
            '-z', '27.5',
            '-y', '-126',
            '-file', robot_description_file,
            '-Y', '3.14159'  # Rotación de 180 grados en el eje Yaw (Z)
        ],
        output='screen'
    )

    
    # Crear el LaunchDescription e incluir las acciones
    ld = LaunchDescription()
    # ld.add_action(set_ignition_model_path_cmd)
    ld.add_action(start_ignition_server_cmd)
    ld.add_action(bridge)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(spawn)
    
    for node in static_tf_nodes:
        ld.add_action(node)
    
    return ld
