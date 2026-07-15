import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'wvcsc_c10_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@example.com',
    description='Synria C10 ROS 2 wrapper around usb_cam with diagnostics.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'camera_watchdog = wvcsc_c10_camera.camera_watchdog:main',
    ]},
)
