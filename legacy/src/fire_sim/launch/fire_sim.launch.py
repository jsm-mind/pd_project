import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('fire_sim'), 'config', 'fire_sim.yaml')
    params = LaunchConfiguration('fire_params_file')
    arg = LaunchConfiguration

    def value(name, value_type):
        return ParameterValue(arg(name), value_type=value_type)

    return LaunchDescription([
        DeclareLaunchArgument('fire_params_file', default_value=default_params),
        DeclareLaunchArgument('min_delay', default_value='30.0'),
        DeclareLaunchArgument('max_delay', default_value='120.0'),
        DeclareLaunchArgument('fires', default_value='true',
                              description='Spawn fires (false for the false alarm scenarios)'),
        DeclareLaunchArgument('fire_index', default_value='-1',
                              description='Candidate index for every fire, -1: random'),
        DeclareLaunchArgument('false_alarm', default_value='false'),
        DeclareLaunchArgument('false_alarm_start', default_value='60.0'),
        DeclareLaunchArgument('false_alarm_duration', default_value='90.0'),

        # name/namespace are given so the launch argument overrides below apply to this node
        # (otherwise launch_ros passes them as '/**', which the node section of the params
        # file overrides).
        # Uses the Gazebo spawn/remove services and /sim/ground_truth_poses bridged by
        # fire_robot_gazebo/sim.launch.py
        Node(
            package='fire_sim',
            executable='fire_generator',
            name='fire_generator',
            namespace='',
            output='screen',
            condition=IfCondition(arg('fires')),
            parameters=[params, {'use_sim_time': True,
                                 'min_delay': value('min_delay', float),
                                 'max_delay': value('max_delay', float),
                                 'fire_index': value('fire_index', int)}],
        ),
        Node(
            package='fire_sim',
            executable='fire_manager',
            output='screen',
            parameters=[params, {'use_sim_time': True}],
        ),
        Node(
            package='fire_sim',
            executable='alarm_simulator',
            name='alarm_simulator',
            namespace='',
            output='screen',
            parameters=[params, {'use_sim_time': True,
                                 'false_alarm': value('false_alarm', bool),
                                 'false_alarm_start': value('false_alarm_start', float),
                                 'false_alarm_duration': value('false_alarm_duration', float)}],
        ),
    ])
