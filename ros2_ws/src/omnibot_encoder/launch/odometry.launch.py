from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='omnibot_encoder',
            executable='encoder_node',
            name='encoder_node',
            output='screen'
        ),
        Node(
            package='omnibot_encoder',
            executable='odometry_node',
            name='odometry_node',
            output='screen'
        )
    ])