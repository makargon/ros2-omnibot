from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Launch complete SLAM system with Foxglove Bridge."""
    
    # Get package directories
    omnibot_perception_dir = get_package_share_directory('omnibot_perception')
    slam_launch = os.path.join(omnibot_perception_dir, 'launch', 'slam.launch.py')
    # Declare launch arguments
    foxglove_enabled = DeclareLaunchArgument(
        'foxglove',
        default_value='true',
        description='Launch Foxglove Bridge'
    )
    foxglove_port = DeclareLaunchArgument(
        'foxglove_port',
        default_value='8765',
        description='WebSocket port for Foxglove Bridge'
    )
    
    ld = LaunchDescription([
        foxglove_enabled,
        foxglove_port,
        
        # Include complete SLAM launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch)
        ),
        
        # Foxglove Bridge
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'port': LaunchConfiguration('foxglove_port'),
                'address': '0.0.0.0',
            }],
            output='screen',
            condition=IfCondition(LaunchConfiguration('foxglove'))
        ),
    ])
    
    return ld
