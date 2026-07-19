import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'wvcsc_arm_task'

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
    description='Alicia-M observation and simulated spraying task.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'spray_task = wvcsc_arm_task.spray_task:main',
        'spray_simulator = wvcsc_arm_task.spray_simulator:main',
        'motion_control = wvcsc_arm_task.motion_control:main',
    ]},
)
