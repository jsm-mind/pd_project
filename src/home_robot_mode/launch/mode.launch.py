from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='home_robot_mode', executable='mode_manager', output='screen'),
        Node(package='home_robot_perception', executable='fire_detector', output='screen'),
        Node(package='home_robot_perception', executable='person_detector', output='screen'),
    ])
