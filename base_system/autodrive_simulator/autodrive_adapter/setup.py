from setuptools import setup
import os
from glob import glob

package_name = 'autodrive_adapter'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='djdis',
    maintainer_email='djdisco33@gmail.com',
    description='Makes the AutoDRIVE simulator look like the car hardware (VESC, IMU, LiDAR) to the race stack.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'autodrive_adapter_node = autodrive_adapter.autodrive_adapter_node:main',
        ],
    },
)
