from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='omnibot_control', executable='actuator_node', name='actuator_node', output='screen'),
        Node(package='omnibot_control', executable='manual_control', name='manual_control', output='screen'),
    ])