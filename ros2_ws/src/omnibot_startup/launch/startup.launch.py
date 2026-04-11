from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    startup_node = Node(
        package='omnibot_startup',
        executable='run',
        name='startup_node',
        output='screen',
    )

    return LaunchDescription([startup_node])