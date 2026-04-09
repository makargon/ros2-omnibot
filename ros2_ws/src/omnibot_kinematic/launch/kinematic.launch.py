from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('omnibot_kinematic')
    params_file = os.path.join(pkg_share, 'config', 'kinematic.yaml')

    kinematic_node = Node(
        package='omnibot_kinematic',
        executable='run',
        name='kinematic_node',
        output='screen',
        # parameters=[params_file],
    )

    return LaunchDescription([kinematic_node])