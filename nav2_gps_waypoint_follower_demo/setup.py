from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav2_gps_waypoint_follower_demo'

# Se copian todos los archivos de models/n1, excepto la carpeta "meshes"
n1_files = [f for f in glob('models/n1/*') if os.path.basename(f) != 'meshes']

# Se obtienen los archivos de la carpeta "meshes" dentro de models/n1
meshes_files_n1 = glob('models/n1/meshes/*')

# Se copian todos los archivos de models/n1, excepto la carpeta "meshes"
n2_files = [f for f in glob('models/n2/*') if os.path.basename(f) != 'meshes']

# Se obtienen los archivos de la carpeta "meshes" dentro de models/n1
meshes_files_n2 = glob('models/n2/meshes/*')

# Se copian todos los archivos de models/n1, excepto la carpeta "meshes"
nn1_files = [f for f in glob('models/nn1/*') if os.path.basename(f) != 'meshes']

# Se obtienen los archivos de la carpeta "meshes" dentro de models/n1
meshes_files_nn1 = glob('models/nn1/meshes/*')

# Se copian todos los archivos de models/n1, excepto la carpeta "meshes"
campusReducido_files = [f for f in glob('models/campusReducido/*') if os.path.basename(f) != 'meshes']

# Se obtienen los archivos de la carpeta "meshes" dentro de models/n1
meshes_files_campusReducido = glob('models/campusReducido/meshes/*')

# Se copian todos los archivos de models/turtlebot_waffle_gps, excepto la carpeta "meshes"
turtlebot_files = [f for f in glob('models/turtlebot_waffle_gps/*') if os.path.basename(f) != 'meshes']

# Se obtienen los archivos de la carpeta "meshes" dentro de models/turtlebot_waffle_gps
meshes_files_turtlebot = glob('models/turtlebot_waffle_gps/meshes/*')

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'models/turtlebot_waffle_gps'),
         turtlebot_files),
        # Copiamos models/n1 sin la carpeta "meshes"
        (os.path.join('share', package_name, 'models/n1'), n1_files),
        # Copiamos la carpeta "meshes" de n1 en otra ubicación: share/<package_name>/models/meshes
        (os.path.join('share', package_name, 'models/n1/meshes'), meshes_files_n1),
        # Copiamos models/n1 sin la carpeta "meshes"
        (os.path.join('share', package_name, 'models/n2'), n2_files),
        # Copiamos la carpeta "meshes" de n1 en otra ubicación: share/<package_name>/models/meshes
        (os.path.join('share', package_name, 'models/n2/meshes'), meshes_files_n2),
        # Copiamos models/n1 sin la carpeta "meshes"
        (os.path.join('share', package_name, 'models/nn1'), nn1_files),
        # Copiamos la carpeta "meshes" de n1 en otra ubicación: share/<package_name>/models/meshes
        (os.path.join('share', package_name, 'models/nn1/meshes'), meshes_files_nn1),
        # Copiamos models/n1 sin la carpeta "meshes"
        (os.path.join('share', package_name, 'models/campusReducido'), campusReducido_files),
        # Copiamos la carpeta "meshes" de n1 en otra ubicación: share/<package_name>/models/meshes
        (os.path.join('share', package_name, 'models/campusReducido/meshes'), meshes_files_campusReducido),
        # Copiamos la carpeta "meshes" de turtlebot_waffle_gps en otra ubicación: share/<package_name>/models/meshes
        (os.path.join('share', package_name, 'models/turtlebot_waffle_gps/meshes'), meshes_files_turtlebot),
    ],
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
