from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    default_field_yaml = os.path.join(
        get_package_share_directory('omnibot_cv'),
        'config',
        'aruco_localization.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/station/camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/station/camera/camera_info'),
        DeclareLaunchArgument('aruco_dictionary', default_value='DICT_4X4_50'),
        DeclareLaunchArgument('marker_length_m', default_value='0.06'),
        DeclareLaunchArgument('robot_marker_id', default_value='0'),
        DeclareLaunchArgument('field_map_yaml', default_value=default_field_yaml),

        Node(
            package='omnibot_cv',
            executable='image_undistort_node',
            name='undistort_station_camera',
            output='screen',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'output_image_topic': '/station/camera/image_undistorted',
            }],
        ),
        Node(
            package='omnibot_cv',
            executable='aruco_pose_node',
            name='aruco_pose_node',
            output='screen',
            parameters=[LaunchConfiguration('field_map_yaml'), {
                'image_topic': '/station/camera/image_undistorted',
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'aruco_dictionary': LaunchConfiguration('aruco_dictionary'),
                'marker_length_m': LaunchConfiguration('marker_length_m'),
                'robot_marker_id': LaunchConfiguration('robot_marker_id'),
            }],
        ),
    ])
