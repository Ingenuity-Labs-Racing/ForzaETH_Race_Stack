from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *[(os.path.join('share', package_name, os.path.basename(os.path.dirname(path))), [path])
          for path in glob('controller/*/*.yaml')],
        (os.path.join('share', package_name, 'evaluation'), glob('evaluation/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ForzaETH',
    maintainer_email='nicolas.baumann@pbl.ee.ethz.ch',
    description='Controller runs whatever f110 controller is desired',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = core.controller_manager:main',
            'controller_evaluation = evaluation.trial_runner:main',
            'controller_evaluation_plot = evaluation.plotting:main',
        ],
    },
)
