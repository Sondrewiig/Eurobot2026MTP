from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'overhead_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch and config files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thanish',
    maintainer_email='tanishgnanasegaram@gmail.com',
    description='Overhead camera vision and world model for Eurobot 2026',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'overhead_camera_node = overhead_control.overhead_camera_node:main',
            'ninja_pose_from_overhead = overhead_control.ninja_pose_from_overhead:main',
            'main_bot_pose_from_overhead = overhead_control.main_bot_pose_from_overhead:main',
            'enemy_pose_from_overhead = overhead_control.enemy_pose_from_overhead:main',
            'operator_gui = overhead_control.operator_gui:main',
        ],
    },
)