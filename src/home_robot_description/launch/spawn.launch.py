import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('home_robot_description')
    xacro_file = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        # Default spawn pose: on the charging pad
        DeclareLaunchArgument('x', default_value='0.6'),
        DeclareLaunchArgument('y', default_value='3.6'),
        DeclareLaunchArgument('z', default_value='0.12'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'robot', '-topic', 'robot_description',
                       '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                       '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': os.path.join(pkg, 'config', 'bridge.yaml'),
                         'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='home_robot_description',
            executable='thermal_sensor.py',
            output='screen',
        ),
    ])
