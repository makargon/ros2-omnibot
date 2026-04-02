from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Launch SLAM with LIDAR perception."""
    
    # Get package directories
    omnibot_perception_dir = get_package_share_directory('omnibot_perception')
    slam_config_file = os.path.join(omnibot_perception_dir, 'config', 'slam.yaml')
    lidar_launch = os.path.join(omnibot_perception_dir, 'launch', 'lidar.launch.py')
    
    ld = LaunchDescription([
        # Include LIDAR perception launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch)
        ),
        
        # SLAM Toolbox node
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_config_file],
            output='screen'
        ),
        
        # Map server for saving/loading maps (optional)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                'yaml_filename': '/tmp/map.yaml',
            }],
            remappings=[
                ('map', 'map'),
                ('map_metadata', 'map_metadata'),
            ],
            output='screen'
        ),
    ])
    
    return ld
