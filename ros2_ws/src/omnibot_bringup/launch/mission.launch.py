from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    mission_name = LaunchConfiguration('mission_name')

    return LaunchDescription([
        DeclareLaunchArgument('mission_name', default_value='demo_mission'),
        LogInfo(msg=['Mission launcher placeholder. Active mission: ', mission_name]),
    ])
