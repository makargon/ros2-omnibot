from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def _self_include(launch_name: str, condition=None, launch_arguments=None):
    launch_path = os.path.join(get_package_share_directory('omnibot_bringup'), 'launch', launch_name)
    kwargs = {
        'launch_description_source': PythonLaunchDescriptionSource(launch_path),
    }
    if condition is not None:
        kwargs['condition'] = condition
    if launch_arguments is not None:
        kwargs['launch_arguments'] = launch_arguments.items()
    return IncludeLaunchDescription(**kwargs)


def generate_launch_description() -> LaunchDescription:
    with_robot = LaunchConfiguration('with_robot')
    with_station = LaunchConfiguration('with_station')
    with_mission = LaunchConfiguration('with_mission')

    return LaunchDescription([
        DeclareLaunchArgument('with_robot', default_value='true'),
        DeclareLaunchArgument('with_station', default_value='true'),
        DeclareLaunchArgument('with_mission', default_value='false'),

        _self_include('robot.launch.py', IfCondition(with_robot)),
        _self_include('station.launch.py', IfCondition(with_station)),
        _self_include('mission.launch.py', IfCondition(with_mission)),
    ])
