from glob import glob
from setuptools import setup

package_name = 'sondre_bot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sondre',
    maintainer_email='sondrewiig333@gmail.com',
    description='Overhead camera board rectification for Eurobot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'overhead_rectifier_node = sondre_bot_vision.overhead_rectifier_node:main',
        ],
    },
)
