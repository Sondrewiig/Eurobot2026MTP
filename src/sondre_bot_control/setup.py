from setuptools import find_packages, setup

package_name = 'sondre_bot_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sondre',
    maintainer_email='your_email@example.com',
    description='ROS 2 camera-based driving logic for Sondre\'s robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_drive = sondre_bot_control.vision_drive:main',
        ],
    },
)