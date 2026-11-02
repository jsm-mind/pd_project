"""Scenario: fire at a candidate with a smoke detector.
Expected: alarm -> ALARM_SWEEP -> FIRE_CONFIRMED -> extinguished -> alarm off.

ros2 launch fire_mission normal.launch.py [gui:=false] [rviz:=true]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

SCENARIO = {
    'fire_index': '4',      # kitchen, north (has a detector)
    'min_delay': '30.0',
    'max_delay': '30.0',
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
