"""Everything in one launch: Gazebo + robot, mode manager + fire detector, contact,
Nav2 + patrol (delayed until Gazebo is up), and a key window (xterm)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAV_DELAY = 15.0    # [s] wait for Gazebo, robot spawn and /clock before Nav2


def include(pkg, name, args=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(pkg), 'launch', name)),
        launch_arguments=(args or {}).items(),
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fire_spot', default_value='random'),
        DeclareLaunchArgument('call', default_value='false'),
        include('home_robot_description', 'sim.launch.py',
                {'fire_spot': LaunchConfiguration('fire_spot')}),
        include('home_robot_mode', 'mode.launch.py'),
        include('home_robot_contact', 'contact.launch.py', {'call': LaunchConfiguration('call')}),
        TimerAction(period=NAV_DELAY, actions=[include('home_robot_nav', 'nav.launch.py')]),
        Node(
            package='home_robot_mode',
            executable='key_trigger',
            prefix='xterm -fa Monospace -fs 12 -geometry 70x8 -T mode_keys -e',
        ),
    ])
