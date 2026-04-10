from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_nodes(context, *args, **kwargs):
    mission_name = LaunchConfiguration('mission_name').perform(context).strip()

    aliases = {
        'demo_mission': 'pickup_delivery_mission',
        'template': 'mission_template',
        'example': 'pickup_delivery_mission',
        'pickup': 'pickup_delivery_mission',
    }
    executable = aliases.get(mission_name, mission_name)

    return [
        Node(
            package='omnibot_mission',
            executable=executable,
            name='mission_runner',
            output='screen',
        )
    ]

def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'mission_name',
            default_value='demo_mission',
            description='Mission script or alias: demo_mission|template|example|pickup|<console_script>',
        ),
        OpaqueFunction(function=_build_nodes),
    ])
