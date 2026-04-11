from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """запуск safety ноды с защитой от препятствий."""
    
    # получаем директории пакетов
    omnibot_perception_dir = get_package_share_directory('omnibot_perception')
    lidar_config_file = os.path.join(omnibot_perception_dir, 'config', 'lidar.yaml')
    tf_config_file = os.path.join(omnibot_perception_dir, 'config', 'tf_broadcaster.yaml')
    safety_config_file = os.path.join(omnibot_perception_dir, 'config', 'safety.yaml')
    

    danger_distance = DeclareLaunchArgument(
        'danger_distance',
        default_value='0.4',
        description='опасная дистанция для аварийной остановки (метры)'
    )
    
    warning_distance = DeclareLaunchArgument(
        'warning_distance',
        default_value='0.55',
        description='дистанция для предупреждения (метры)'
    )
    
    safety_angle = DeclareLaunchArgument(
        'safety_angle',
        default_value='50.0',
        description='угол обзора безопасности в градусах'
    )
    
    enable_logging = DeclareLaunchArgument(
        'enable_logging',
        default_value='true',
        description='включить подробное логирование'
    )
    
    ld = LaunchDescription([        
        # safety нода (останавливает при препятствиях)
        Node(
            package='omnibot_perception',
            executable='safety_node',
            name='safety_node',
            parameters=[safety_config_file],
            remappings=[
                ('cmd_vel', 'cmd_vel'),           # входная команда скорости
                ('cmd_vel_safe', 'cmd_vel_safe'), # безопасная команда
                ('scan', 'scan'),                 # данные с лидара
            ],
            output='screen'
        ),
    ])
    
    return ld