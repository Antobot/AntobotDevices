from setuptools import find_packages, setup

package_name = 'antobot_devices_joy'
import os  # Add this line
from glob import glob

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Huaide Wang',
    maintainer_email='huaide.wang@nicecart.ai',
    description='The antobot_devices_joy package, which contains the code for joy write by ourselves',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_elrs_node = antobot_devices_joy.joy_elrs_node:main',
            'joy_sbus_node = antobot_devices_joy.joy_sbus_node:main',
        ],
    },
)
