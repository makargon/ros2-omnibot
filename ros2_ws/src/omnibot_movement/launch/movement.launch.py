from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('omnibot_movement')
    params_file = os.path.join(pkg_share, 'config', 'movement.yaml')

    movement_node = Node(
        package='omnibot_movement',
        executable='run',
        name='movement_node',
        output='screen',
        parameters=[ParameterFile(params_file, allow_substs=True)],
    )

    return LaunchDescription([movement_node])