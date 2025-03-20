import os
import random
import math
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_sheep_nodes(num_sheep=30, max_radius=10.0, min_distance=2.0, max_attempts=20):
    """
    Genera nodos ROS 2 para spawnear ovejas en Ignition Gazebo, asegurando que no colisionen.
    
    Parámetros:
    - num_sheep (int): Número de ovejas a spawnear.
    - max_radius (float): Radio máximo del área donde se distribuirán.
    - min_distance (float): Distancia mínima entre ovejas.
    - max_attempts (int): Intentos máximos antes de descartar una oveja.

    Retorna:
    - Lista de nodos `Node()` para lanzar las ovejas en Gazebo.
    """
    gps_wpf_dir = get_package_share_directory("nav2_gps_waypoint_follower_demo")
    sheep_model_path = os.path.join(gps_wpf_dir, "models", "sheep", "model.sdf")

    center_x = -169 - 10  # Centro del círculo (10m del robot)
    center_y = -126

    sheep_positions = []
    sheep_nodes = []

    for i in range(num_sheep):
        attempts = 0
        while attempts < max_attempts:
            # Distribución radial no uniforme
            radius_factor = random.uniform(0, 1)
            radius = max_radius * (1 - math.sqrt(radius_factor))

            # Ángulo aleatorio
            angle = random.uniform(0, 2 * math.pi)

            # Posiciones alrededor del centro
            x_pos = radius * math.cos(angle) + center_x
            y_pos = radius * math.sin(angle) + center_y

            # Verificar si está demasiado cerca de otras ovejas
            too_close = any(
                math.sqrt((x_pos - px) ** 2 + (y_pos - py) ** 2) < min_distance
                for px, py in sheep_positions
            )

            if not too_close:
                sheep_positions.append((x_pos, y_pos))
                break  # Sale del bucle si encontró una buena posición
            attempts += 1

        # Si después de varios intentos no encontró espacio, la oveja no se spawnea
        if attempts == max_attempts:
            print(f"Oveja {i} descartada por falta de espacio.")
            continue

        # 🔹 Orientación: 8 valores posibles (múltiplos de 45°)
        orientation = random.choice([
            0, math.pi / 4, math.pi / 2, 3 * math.pi / 4,
            math.pi, -3 * math.pi / 4, -math.pi / 2, -math.pi / 4
        ])

        # Crear nodo para la oveja
        sheep_spawn = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', f'sheep_{i}',
                '-x', str(x_pos),
                '-y', str(y_pos),
                '-z', '27.5',
                '-file', sheep_model_path,
                '-Y', str(orientation)
            ],
            output='screen'
        )
        sheep_nodes.append(sheep_spawn)

    return sheep_nodes
