from setuptools import setup

package_name = 'home_robot_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeongseunggeon11',
    maintainer_email='blueciki@gmail.com',
    description='Thermal-array perception (fire detection) for the unit-1 home fire robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fire_detector = home_robot_perception.fire_detector:main',
        ],
    },
)
