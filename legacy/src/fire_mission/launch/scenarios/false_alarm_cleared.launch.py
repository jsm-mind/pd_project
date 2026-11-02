"""Scenario: alarm without a fire, off before the first sweep round ends.
Expected: ALARM_SWEEP once -> owner notice -> PATROL.

ros2 launch fire_mission false_alarm_cleared.launch.py [gui:=false] [rviz:=true]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

SCENARIO = {
    'fires': 'false',
    'false_alarm': 'true',
    'false_alarm_start': '60.0',
    'false_alarm_duration': '90.0',
}


def generate_launch_description():
    full = os.path.join(get_package_share_directory('fire_mission'), 'launch', 'full.launch.py')
    args = {'gui': LaunchConfiguration('gui'), 'rviz': LaunchConfiguration('rviz'), **SCENARIO}
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(full),
                                 launch_arguments=args.items()),
    ])
