"""Scenario: fire at a candidate without a smoke detector.
Expected: no alarm, the fire is found by the normal patrol.

ros2 launch fire_mission no_detector.launch.py [gui:=false] [rviz:=true]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

SCENARIO = {
    'fire_index': '7',      # hall (no detector)
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
