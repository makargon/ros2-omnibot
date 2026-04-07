from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('omnibot_cv')
    camera_launch = os.path.join(pkg_share, 'launch', 'camera_capture.launch.py')
    aruco_launch = os.path.join(pkg_share, 'launch', 'aruco_localization.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(camera_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(aruco_launch)),
    ])
