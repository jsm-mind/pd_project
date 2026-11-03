from setuptools import setup

package_name = 'home_robot_contact'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/contact.yaml']),
        ('share/' + package_name + '/launch', ['launch/contact.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeongseunggeon11',
    maintainer_email='blueciki@gmail.com',
    description='119 phone call (Twilio) and guardian/resident terminal contact',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'contact = home_robot_contact.contact:main',
            'guidance = home_robot_contact.guidance:main',
            'status_ring = home_robot_contact.status_ring:main',
        ],
    },
)
