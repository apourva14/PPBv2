from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'ppb_vio'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config', 'estimators'),
            glob('config/estimators/*.yaml')),
        (os.path.join('share', package_name, 'config', 'platforms'),
            glob('config/platforms/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy', 'pyyaml'],
    zip_safe=True,
    maintainer='Yiyuan Lin',
    maintainer_email='yl3663@cornell.edu',
    description='PPBv2 VIO integration (/vio/odom adapter, stream monitor, estimator launch)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vio_odom_adapter = ppb_vio.vio_odom_adapter:main',
            'vio_stream_monitor = ppb_vio.vio_stream_monitor:main',
        ],
    },
)
