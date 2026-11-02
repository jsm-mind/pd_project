"""Whole system: simulation, Nav2, fire simulation, alarm, detection and the mission state machine.

The defaults give random fires. launch/scenarios/*.launch.py fix the arguments for each
test scenario.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            LogInfo, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Passed through to fire_sim.launch.py
FIRE_SIM_ARGS = ['min_delay', 'max_delay', 'fires', 'fire_index',
                 'false_alarm', 'false_alarm_start', 'false_alarm_duration']


def include(package, launch_file, args=None):
    """Include a launch file in its own scope, so launch arguments with the same name in
    different files (e.g. 'params_file') do not leak into each other."""
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    return GroupAction(scoped=True, actions=[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                 launch_arguments=(args or {}).items())])


def generate_launch_description():
    # Wait until Gazebo has spawned the robot (create exits), however long the GUI takes
    # to load, so Nav2 and the rest do not compete with Gazebo during its startup.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'fire_robot',
                   '-x', '4.5', '-y', '2.0', '-z', '0.01', '-Y', '0.0'],
    )
    rest = [
        LogInfo(msg='Robot spawned, starting Nav2, fire simulation, detection and mission'),
        include('fire_robot_navigation', 'navigation.launch.py',
                {'rviz': LaunchConfiguration('rviz')}),
        include('fire_sim', 'fire_sim.launch.py',
                {name: LaunchConfiguration(name) for name in FIRE_SIM_ARGS}),
        include('fire_detection', 'fire_detection.launch.py'),
        include('fire_mission', 'mission.launch.py',
                {'report_config': LaunchConfiguration('report_config')}),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('min_delay', default_value='30.0'),
        DeclareLaunchArgument('max_delay', default_value='120.0'),
        DeclareLaunchArgument('fires', default_value='true'),
        DeclareLaunchArgument('fire_index', default_value='-1'),
        DeclareLaunchArgument('false_alarm', default_value='false'),
        DeclareLaunchArgument('false_alarm_start', default_value='60.0'),
        DeclareLaunchArgument('false_alarm_duration', default_value='90.0'),
        DeclareLaunchArgument('report_config', default_value='~/.config/fire_robot/report.yaml'),
        DeclareLaunchArgument('robot_model', default_value='fire_robot_unit1.urdf.xacro'),

        include('fire_robot_gazebo', 'sim.launch.py',
                {'gui': LaunchConfiguration('gui'), 'spawn': 'false',
                 'robot_model': LaunchConfiguration('robot_model')}),
        spawn,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn,
            on_exit=lambda event, _context: rest if event.returncode == 0 else [
                LogInfo(msg='Robot spawn failed, not starting the rest')])),
    ])
