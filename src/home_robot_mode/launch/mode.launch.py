from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='home_robot_mode', executable='mode_manager', output='screen'),
    ])
