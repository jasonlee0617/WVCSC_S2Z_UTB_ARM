import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'wvcsc_uav_gateway'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@example.com',
    description='Mock and replay UAV disease-tree mission gateway.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'mock_uav_gateway = wvcsc_uav_gateway.mock_uav_gateway:main',
        'replay_uav_gateway = wvcsc_uav_gateway.replay_uav_gateway:main',
    ]},
)
