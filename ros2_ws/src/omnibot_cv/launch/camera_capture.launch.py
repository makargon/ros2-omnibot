from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _camera_node(context):
    image_width = int(LaunchConfiguration('image_width').perform(context))
    image_height = int(LaunchConfiguration('image_height').perform(context))

    return [
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name=LaunchConfiguration('camera_name'),
            output='screen',
            parameters=[{
                'video_device': LaunchConfiguration('video_device'),
                'camera_frame_id': LaunchConfiguration('frame_id'),
                'camera_info_url': LaunchConfiguration('camera_info_url'),
                'pixel_format': LaunchConfiguration('pixel_format'),
                'io_method': LaunchConfiguration('io_method'),
                'image_size': [image_width, image_height],
            }],
            remappings=[
                ('image_raw', LaunchConfiguration('image_topic')),
                ('camera_info', LaunchConfiguration('camera_info_topic')),
            ],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('frame_id', default_value='camera_optical_frame'),
        DeclareLaunchArgument('camera_info_url', default_value=''),
        DeclareLaunchArgument('pixel_format', default_value='YUYV'),
        DeclareLaunchArgument('io_method', default_value='read'),
        DeclareLaunchArgument('image_width', default_value='640'),
        DeclareLaunchArgument('image_height', default_value='480'),
        DeclareLaunchArgument('image_topic', default_value='image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='camera_info'),
        OpaqueFunction(function=_camera_node),
    ])
