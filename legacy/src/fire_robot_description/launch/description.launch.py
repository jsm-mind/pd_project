import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    urdf_dir = os.path.join(get_package_share_directory('fire_robot_description'), 'urdf')
    xacro_file = PathJoinSubstitution([urdf_dir, LaunchConfiguration('robot_model')])
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot_model', default_value='fire_robot_unit1.urdf.xacro',
                              description='Body xacro file in fire_robot_description/urdf'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
