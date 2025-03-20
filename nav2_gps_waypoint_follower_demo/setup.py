from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav2_gps_waypoint_follower_demo'

# Función para obtener archivos de modelos excluyendo "meshes" y listando "meshes" por separado
def get_model_files(model_path):
    all_files = glob(os.path.join(model_path, '*'))
    meshes_path = os.path.join(model_path, 'meshes')
    files_without_meshes = [f for f in all_files if os.path.basename(f) != 'meshes']
    meshes_files = glob(os.path.join(meshes_path, '*')) if os.path.exists(meshes_path) else []
    return files_without_meshes, meshes_files

# Lista de modelos
models = ['n1', 'campusReducido', 'map', 'turtlebot_waffle_gps', 'sheep', 'campusReducidoUp']

# Generamos la lista de data_files automáticamente
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*')),
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
]

# Agregar modelos dinámicamente
for model in models:
    model_path = os.path.join('models', model)
    model_files, meshes_files = get_model_files(model_path)

    # Agregar archivos del modelo (excepto "meshes")
    data_files.append((os.path.join('share', package_name, model_path), model_files))

    # Agregar archivos de la carpeta "meshes" si existen
    if meshes_files:
        data_files.append((os.path.join('share', package_name, model_path, 'meshes'), meshes_files))

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='pedro.gonzalez@eia.edu.co',
    description='Demo package for following GPS waypoints with nav2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'logged_waypoint_follower = nav2_gps_waypoint_follower_demo.logged_waypoint_follower:main',
            'interactive_waypoint_follower = nav2_gps_waypoint_follower_demo.interactive_waypoint_follower:main',
            'gps_waypoint_logger = nav2_gps_waypoint_follower_demo.gps_waypoint_logger:main'
        ],
    },
)
