# 中文说明：安装 C10 相机节点、launch 文件和相机参数资源。
# 设备路径、图像话题和相机信息由 launch/YAML 公开配置，不在 setup 中硬编码。
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
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@example.com',
    description='Synria C10 ROS 2 wrapper around usb_cam with diagnostics.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'camera_watchdog = wvcsc_c10_camera.camera_watchdog:main',
    ]},
)
