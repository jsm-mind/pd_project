import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

SECRET = os.path.expanduser('~/proj_ws/twilio.yaml')   # Twilio credentials, git-ignored


def generate_launch_description():
    params = [os.path.join(get_package_share_directory('home_robot_contact'),
                           'config', 'contact.yaml')]
    if os.path.exists(SECRET):
        params.append(SECRET)
    # real call only when asked for at launch, so tests never dial
    params.append({'call_enabled': ParameterValue(LaunchConfiguration('call'), value_type=bool)})
    return LaunchDescription([
        DeclareLaunchArgument('call', default_value='false',
                              description='true: really dial call_number'),
        DeclareLaunchArgument('relay_port', default_value='8080',
                              description='fire station relay screen port'),
        Node(package='home_robot_contact', executable='contact',
             parameters=params, output='screen', emulate_tty=True),
        Node(package='home_robot_contact', executable='guidance', output='screen', emulate_tty=True),
        Node(package='home_robot_contact', executable='status_ring', output='screen',
             emulate_tty=True),
        Node(package='home_robot_contact', executable='sound', output='screen', emulate_tty=True),
        Node(package='home_robot_contact', executable='control_relay', output='screen',
             parameters=[{'port': ParameterValue(LaunchConfiguration('relay_port'), value_type=int)}]),
    ])
