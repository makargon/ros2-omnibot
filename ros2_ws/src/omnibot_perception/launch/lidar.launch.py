from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Launch LIDAR perception nodes."""
    
    # Get package directories
    omnibot_perception_dir = get_package_share_directory('omnibot_perception')
    lidar_config_file = os.path.join(omnibot_perception_dir, 'config', 'lidar.yaml')
    tf_config_file = os.path.join(omnibot_perception_dir, 'config', 'tf_broadcaster.yaml')
    
    # Declare launch arguments
    lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for LIDAR connection'
    )
    
    ld = LaunchDescription([
        lidar_port,
        
        # LIDAR node using rplidar_ros package
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            parameters=[lidar_config_file],
            remappings=[
                ('scan', 'scan'),
            ],
            output='screen'
        ),
        
        # TF Broadcaster node
        Node(
            package='omnibot_perception',
            executable='tf_broadcaster',
            name='tf_broadcaster',
            parameters=[tf_config_file],
            output='screen'
        ),
    ])
    
    return ld
