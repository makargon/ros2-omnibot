from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_nodes(context, *args, **kwargs):
    mission = LaunchConfiguration('mission').perform(context).strip()
    target_marker_id = int(LaunchConfiguration('target_marker_id').perform(context).strip())
    target_marker_ids_csv = LaunchConfiguration('target_marker_ids').perform(context).strip()
    prefer_nearest_marker = LaunchConfiguration('prefer_nearest_marker').perform(context).strip().lower() in ('1', 'true', 'yes', 'on')

    # Удобные алиасы
    aliases = {
        'template': 'mission_template',
        'example': 'pickup_delivery_mission',
        'pickup': 'pickup_delivery_mission',
    }
    executable = aliases.get(mission, mission)

    params = []
    if executable == 'pickup_delivery_mission':
        marker_ids = []
        if target_marker_ids_csv:
            marker_ids = [int(v.strip()) for v in target_marker_ids_csv.split(',') if v.strip()]
        params.append({
            'target_marker_id': target_marker_id,
            'target_marker_ids': marker_ids,
            'prefer_nearest_marker': prefer_nearest_marker,
        })

    return [
        Node(
            package='omnibot_mission',
            executable=executable,
            name='mission_runner',
            parameters=params,
            output='screen',
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'mission',
            default_value='example',
            description='Mission executable name or alias: template|example|pickup|<console_script>',
        ),
        DeclareLaunchArgument('target_marker_id', default_value='7'),
        DeclareLaunchArgument('target_marker_ids', default_value=''),
        DeclareLaunchArgument('prefer_nearest_marker', default_value='true'),
        OpaqueFunction(function=_build_nodes),
    ])
