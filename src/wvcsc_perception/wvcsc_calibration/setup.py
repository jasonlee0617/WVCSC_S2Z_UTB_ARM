# 中文说明：安装标定 Python 模块、launch、YAML 和标定辅助命令。
# 采集器需要交互终端；setup 只负责安装，不自动启动真实硬件。
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
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml') + glob('config/*.calib')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
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
        'visualize_aruco_marker = wvcsc_calibration.visualize_aruco_marker:main',
        'export_handeye = wvcsc_calibration.calibration_io:main',
        'auto_calibration_collector = '
        'wvcsc_calibration.auto_calibration_collector:main',
    ]},
)
