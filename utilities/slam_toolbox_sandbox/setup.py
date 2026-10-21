from setuptools import setup
import os
from glob import glob

package_name = 'slam_toolbox_sandbox'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'models'), glob('models/*.sdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yawar Ashraf',
    maintainer_email='yawar1ashraf@gmail.com',
    description='Standalone Gazebo sandbox for testing SLAM Toolbox',
    license='MIT',
    entry_points={
        'console_scripts': [
            'png_to_world = slam_toolbox_sandbox.png_to_world:main',
            'pose_error = slam_toolbox_sandbox.pose_error:main',
            'autodrive = slam_toolbox_sandbox.autodrive:main',
        ],
    },
)
