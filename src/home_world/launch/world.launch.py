import os
import random

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Fire-prone spots in the house: name -> (x, y, z of burning surface)
FIRE_SPOTS = {
    'stove_1': (1.15, 6.20, 1.05),
    'stove_2': (3.95, 6.40, 1.05),
    'tv_stand': (4.95, 2.60, 0.60),
    'nightstand_bedroom1': (10.50, 0.40, 0.60),
    'nightstand_bedroom2': (10.50, 5.30, 0.60),
    'utility_room': (4.80, 7.10, 0.10),
}


def spawn_fire(context):
    spot = LaunchConfiguration('fire_spot').perform(context)
    if spot == 'random':
        spot = random.choice(list(FIRE_SPOTS))
    x, y, z = FIRE_SPOTS[spot]
    return [
        LogInfo(msg=f'Fire spot: {spot} ({x}, {y}, {z})'),
        # the fire breaks out during the patrol (or on the alarm key)
        Node(package='home_world', executable='fire_sim.py',
             parameters=[{'fire_spot': spot, 'position': [x, y, z]}], output='screen'),
        Node(package='home_world', executable='fire_alarm_sim.py',
             parameters=[{'fire_spot': spot}], output='screen'),
    ]


def generate_launch_description():
    pkg = get_package_share_directory('home_world')
    world = os.path.join(pkg, 'worlds', 'house.sdf')
    models = os.path.join(pkg, 'models')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world}'}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('fire_spot', default_value='random',
                              choices=['random'] + list(FIRE_SPOTS)),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                               models + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
        gz_sim,
        OpaqueFunction(function=spawn_fire),
        Node(package='home_world', executable='resident_sim.py', output='screen'),
    ])
