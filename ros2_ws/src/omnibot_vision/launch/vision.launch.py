from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory('omnibot_vision')
    camera_config = os.path.join(package_share, 'config', 'camera.yaml')
    aruco_config = os.path.join(package_share, 'config', 'aruco.yaml')
    calibration_file = os.path.join(package_share, 'config', 'calibration.yaml')
    field_map_file = os.path.join(package_share, 'config', 'field_map_example.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='0', description='USB camera device index or path'),
        DeclareLaunchArgument('frame_id', default_value='camera_link', description='Camera frame id'),
        DeclareLaunchArgument('calibration_file', default_value=calibration_file, description='Camera calibration YAML file'),
        DeclareLaunchArgument('field_map_file', default_value=field_map_file, description='Field ArUco map YAML file'),
        Node(
            package='omnibot_vision',
            executable='camera_node',
            name='camera_node',
            parameters=[
                camera_config,
                {
                    'device': LaunchConfiguration('device'),
                    'frame_id': LaunchConfiguration('frame_id'),
                    'calibration_file': LaunchConfiguration('calibration_file'),
                },
            ],
            output='screen',
        ),
        Node(
            package='omnibot_vision',
            executable='aruco_pose_node',
            name='aruco_pose_node',
            parameters=[
                aruco_config,
                {
                    'marker_map_file': LaunchConfiguration('field_map_file'),
                },
            ],
            output='screen',
        ),
    ])