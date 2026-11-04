from setuptools import setup

package_name = 'home_robot_mode'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mode.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeongseunggeon11',
    maintainer_email='blueciki@gmail.com',
    description='Mode management (patrol / charging / emergency) for the unit-1 home fire robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mode_manager = home_robot_mode.mode_manager:main',
            'key_trigger = home_robot_mode.key_trigger:main',
            'alarm_receiver = home_robot_mode.alarm_receiver:main',
            'scenario = home_robot_mode.scenario:main',
            'robot_status = home_robot_mode.robot_status:main',
        ],
    },
)
