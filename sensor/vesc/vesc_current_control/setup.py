import os
from glob import glob

from setuptools import setup

package_name = 'vesc_current_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py') + glob('launch/*.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='heedo7425',
    maintainer_email='heedo7425@gmail.com',
    description='VESC 속도 PID -> 전류 변환 노드 + 벤치 테스트 툴',
    license='MIT',
    entry_points={
        'console_scripts': [
            'speed_pid_to_current = vesc_current_control.speed_pid_to_current:main',
            'bench_gui = vesc_current_control.bench_gui:main',
            'bench_console = vesc_current_control.bench_console:main',
            'fake_vesc = vesc_current_control.fake_vesc:main',
        ],
    },
)
