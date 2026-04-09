from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('omnibot_actuator')
    params_file = os.path.join(pkg_share, 'config', 'actuator.yaml')

    actuator_node = Node(
        package='omnibot_actuator',
        executable='run',
        name='actuator_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([actuator_node])