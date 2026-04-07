from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _include(package_name: str, launch_name: str, condition=None, launch_arguments=None):
    launch_path = os.path.join(get_package_share_directory(package_name), 'launch', launch_name)
    kwargs = {
        'launch_description_source': PythonLaunchDescriptionSource(launch_path),
    }
    if condition is not None:
        kwargs['condition'] = condition
    if launch_arguments is not None:
        kwargs['launch_arguments'] = launch_arguments.items()
    return IncludeLaunchDescription(**kwargs)


def generate_launch_description() -> LaunchDescription:
    with_station_camera = LaunchConfiguration('with_station_camera')
    with_cv = LaunchConfiguration('with_cv')
    with_foxglove = LaunchConfiguration('with_foxglove')
    foxglove_port = LaunchConfiguration('foxglove_port')

    return LaunchDescription([
        DeclareLaunchArgument('with_station_camera', default_value='true'),
        DeclareLaunchArgument('with_cv', default_value='true'),
        DeclareLaunchArgument('with_foxglove', default_value='true'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument('station_camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument('station_camera_pixel_format', default_value='YUYV'),
        DeclareLaunchArgument('station_camera_io_method', default_value='read'),
        DeclareLaunchArgument('station_calibration_yaml', default_value=''),
        DeclareLaunchArgument('station_camera_topic', default_value='/station/camera/image_raw'),

        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'port': LaunchConfiguration('foxglove_port'),
                'address': '0.0.0.0',
            }],
            output='screen',
            condition=IfCondition(with_foxglove),
        ),
        _include(
            'omnibot_cv',
            'camera_capture.launch.py',
            IfCondition(with_station_camera),
            {
                'camera_name': 'station_camera',
                'frame_id': 'station_camera_optical_frame',
                'video_device': LaunchConfiguration('station_camera_device'),
                'pixel_format': LaunchConfiguration('station_camera_pixel_format'),
                'io_method': LaunchConfiguration('station_camera_io_method'),
                'image_width': LaunchConfiguration('station_camera_width'),
                'image_height': LaunchConfiguration('station_camera_height'),
                'camera_info_url': LaunchConfiguration('station_calibration_yaml'),
                'image_topic': LaunchConfiguration('station_camera_topic'),
                'camera_info_topic': '/station/camera/camera_info',
            },
        ),
        LogInfo(msg=['Station camera device: ', LaunchConfiguration('station_camera_device')]),
        LogInfo(msg=['Station camera pixel format: ', LaunchConfiguration('station_camera_pixel_format')]),
        LogInfo(msg=['Station camera io_method: ', LaunchConfiguration('station_camera_io_method')]),
        LogInfo(msg=['Station camera width: ', LaunchConfiguration('station_camera_width')]),
        LogInfo(msg=['Station camera height: ', LaunchConfiguration('station_camera_height')]),
        _include(
            'omnibot_cv',
            'aruco_localization.launch.py',
            IfCondition(with_cv),
            {
                'image_topic': LaunchConfiguration('station_camera_topic'),
                'camera_info_topic': '/station/camera/camera_info',
            },
        ),

        LogInfo(msg='Path planning node is reserved for future integration.'),
    ])
