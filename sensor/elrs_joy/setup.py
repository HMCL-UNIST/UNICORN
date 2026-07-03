import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'elrs_joy'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'udev'), glob('udev/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HyungJun Kim',
    maintainer_email='hjkim@unist.ac.kr',
    description='ExpressLRS receiver to sensor_msgs/Joy converter',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'elrs_joy_node = elrs_joy.elrs_joy_node:main',
        ],
    },
)
