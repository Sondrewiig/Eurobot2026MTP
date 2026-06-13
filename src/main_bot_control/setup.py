from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'main_bot_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            os.path.join('share', package_name),
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob(os.path.join(package_name, 'config', '*.yaml')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sondre',
    maintainer_email='your_email@example.com',
    description="ROS 2 camera-based driving logic for the Eurobot main robot.",
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_drive = main_bot_control.vision_drive:main',
            'aruco_detect = main_bot_control.aruco_detect:main',
            'tag_localization = main_bot_control.tag_localization:main',
            'ground_truth_pose = main_bot_control.ground_truth_pose:main',
            'pose_compare = main_bot_control.pose_compare:main',
            'telemetry_console = main_bot_control.telemetry_console:main',
            'drive_mode_mux = main_bot_control.drive_mode_mux:main',
            'mode_switch_gui = main_bot_control.mode_switch_gui:main',
            'pose_fuser = main_bot_control.pose_fuser:main',
            'overhead_pose_sim = main_bot_control.overhead_pose_sim:main',
            'zed_left_splitter = main_bot_control.zed_left_splitter:main',
            'opencr_bridge = main_bot_control.opencr_bridge:main',
            'opencr_bridge_sim = main_bot_control.opencr_bridge_sim:main',
            'stereo_capture = main_bot_control.stereo_capture:main',
            'stereo_calibrate = main_bot_control.stereo_calibrate:main',
            'stereo_rectify = main_bot_control.stereo_rectify:main',
            'pose_logger = main_bot_control.pose_logger:main',
            'pose_fuser_sim = main_bot_control.pose_fuser_sim:main',
            'actuator_bridge = main_bot_control.actuator_bridge:main',
            'rbpi_metrics = main_bot_control.rbpi_metrics:main',
        ],
    },
)