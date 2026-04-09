from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('omnibot_teleop')
    params_file = os.path.join(pkg_share, 'config', 'teleop.yaml')

    teleop_node = Node(
        package='omnibot_teleop',
        executable='run',
        name='teleop_node',
        output='screen',
        parameters=[ParameterFile(params_file, allow_substs=True)],
    )

    return LaunchDescription([teleop_node])