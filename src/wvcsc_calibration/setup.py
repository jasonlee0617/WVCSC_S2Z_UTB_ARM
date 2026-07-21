import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'wvcsc_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'xacro'), glob('xacro/*.xacro')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@example.com',
    description='C10 intrinsics and Alicia-M eye-in-hand calibration integration.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'marker_tf = wvcsc_calibration.marker_tf:main',
        'export_handeye = wvcsc_calibration.calibration_io:main',
        'auto_calibration_collector = '
        'wvcsc_calibration.auto_calibration_collector:main',
        'aruco_tf_broadcaster = '
        'wvcsc_calibration.aruco_tf_broadcaster:main',
    ]},
)
