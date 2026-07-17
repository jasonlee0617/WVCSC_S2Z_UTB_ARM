import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'wvcsc_visual_servo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'THIRD_PARTY.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@example.com',
    description='Alicia-M RGB image-based visual servo using MoveIt Servo.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'visual_servo = wvcsc_visual_servo.visual_servo_node:main',
    ]},
)
