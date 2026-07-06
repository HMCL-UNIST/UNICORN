from setuptools import setup

package_name = 'odom_calib'

setup(
    name=package_name,
    version='0.0.1',
    # The importable analysis library (bag_io, calibrate, fusion, gt, ...) plus
    # the online ROS node all live under odom_calib/. The offline entry points
    # (run.py, analyze_bag.sh) stay as standalone dev scripts at the repo root
    # and are intentionally NOT installed as ROS executables.
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='js',
    maintainer_email='kwonc@unist.ac.kr',
    description='VESC odometry gain ID: offline rosbag analysis + online passive tuner.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'online_vesc_tuner = odom_calib.online_tuner_node:main',
        ],
    },
)
