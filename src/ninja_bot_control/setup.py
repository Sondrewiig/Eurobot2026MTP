from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ninja_bot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thanish',
    maintainer_email='tanishgnanasegaram@gmail.com',
    description='ROS 2 package for Eurobot 2026 Ninja SIMA',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'esp32_bridge = ninja_bot_control.esp32_bridge:main',
            'crate_detector = ninja_bot_control.crate_detector_node:main',
            'go_to_point = ninja_bot_control.go_to_point_node:main',
            'crate_align = ninja_bot_control.crate_align_node:main',
            'ninja_mission = ninja_bot_control.ninja_mission_node:main',
        ],
    },
)