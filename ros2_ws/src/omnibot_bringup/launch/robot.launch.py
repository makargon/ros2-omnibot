from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
    with_control = LaunchConfiguration('with_control')
    with_lidar = LaunchConfiguration('with_lidar')
    with_robot_camera = LaunchConfiguration('with_robot_camera')
    with_foxglove = LaunchConfiguration('with_foxglove')
    robot_camera_device = LaunchConfiguration('robot_camera_device')

    return LaunchDescription([
        DeclareLaunchArgument('with_control', default_value='true'),
        DeclareLaunchArgument('with_lidar', default_value='true'),
        DeclareLaunchArgument('with_robot_camera', default_value='true'),
        DeclareLaunchArgument('with_foxglove', default_value='true'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument('robot_camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument('robot_calibration_yaml', default_value=''),

        _include('omnibot_control', 'motor_controller.launch.py', IfCondition(with_control)),
        _include('omnibot_control', 'servo_controller.launch.py', IfCondition(with_control)),
        _include('omnibot_perception', 'lidar.launch.py', IfCondition(with_lidar)),
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
            IfCondition(with_robot_camera),
            {
                'camera_name': 'robot_camera',
                'frame_id': 'robot_camera_optical_frame',
                'video_device': robot_camera_device,
                'camera_info_url': LaunchConfiguration('robot_calibration_yaml'),
                'image_topic': '/robot/camera/image_raw',
                'camera_info_topic': '/robot/camera/camera_info',
            },
        ),
    ])
