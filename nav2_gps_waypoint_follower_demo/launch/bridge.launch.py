from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            # Comando de velocidad (ROS 2 -> Ignition)
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            # Odometría (Ignition -> ROS 2)
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            # TF (Ignition -> ROS 2)
            '/odom/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            # Reloj (Ignition -> ROS 2)
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            # Estados de las articulaciones (Ignition -> ROS 2)
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            # Lidar (Ignition -> ROS 2)
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            # IMU (Ignition -> ROS 2)
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            # Cámara (Ignition -> ROS 2)
            '/camera/rgb/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/rgb/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            # GPS (Ignition -> ROS 2) - Corrected message type
            '/gps/fix@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
        ],
        remappings=[
            ("/odom/tf", "tf"),
        ],
        output='screen'
    )

    map_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']
    )

    return LaunchDescription([
        bridge,
        map_static_tf,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
    ])
