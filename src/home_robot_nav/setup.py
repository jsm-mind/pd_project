from glob import glob

from setuptools import setup

package_name = 'home_robot_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeongseunggeon11',
    maintainer_email='blueciki@gmail.com',
    description='Pre-built map, Nav2 config and patrol / return-to-charger for the unit-1 home fire robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'patrol = home_robot_nav.patrol:main',
            'evacuation = home_robot_nav.evacuation:main',
            'suppression = home_robot_nav.suppression:main',
        ],
    },
)
