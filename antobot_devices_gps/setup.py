from setuptools import find_packages, setup

package_name = 'antobot_devices_gps'
import os  # Add this line
from glob import glob

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+ '/launch',['launch/gps_f9p.launch.py']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antoscout-orin',
    maintainer_email='zhuang.zhou@antobot.ai',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_manager = antobot_devices_gps.gpsManager:main',
            'gps_corrections= antobot_devices_gps.gps_corrections:main',
            'gps_movingbase= antobot_devices_gps.gps_movingbase:main'
        ],
    },
)
