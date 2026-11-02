import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('home_world'), 'launch', 'world.launch.py')),
        launch_arguments={'fire_spot': LaunchConfiguration('fire_spot')}.items(),
    )
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('home_robot_description'), 'launch', 'spawn.launch.py')),
    )
    return LaunchDescription([
        DeclareLaunchArgument('fire_spot', default_value='random'),
        world,
        robot,
    ])
