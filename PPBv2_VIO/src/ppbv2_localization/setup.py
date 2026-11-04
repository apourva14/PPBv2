from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'ppbv2_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'variants'), glob('config/variants/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Apurva Singh',
    maintainer_email='as4642@cornell.edu',
    description='PPBv2 localization: VIO + wheel + IMU (+ GNSS) fusion with robot_localization',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vio_gate = ppbv2_localization.vio_gate_node:main',
        ],
    },
)
