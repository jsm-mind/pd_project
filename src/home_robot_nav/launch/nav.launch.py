import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('home_robot_nav')
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': os.path.join(pkg, 'maps', 'house.yaml'),
            'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'true',
        }.items(),
    )
    patrol = Node(
        package='home_robot_nav',
        executable='patrol',
        # no sim time: avoids processing /clock at physics rate; goals use stamp 0 (latest TF)
        parameters=[os.path.join(pkg, 'config', 'patrol.yaml')],
        output='screen',
    )
    return LaunchDescription([bringup, patrol])
