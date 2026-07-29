# 中文说明：安装视觉伺服 Python 模块及其 ROS 2 可执行入口。
# 节点入口与纯算法模块分离，参数和 Action 话题由 launch/config 提供。
from setuptools import find_packages, setup


package_name = 'wvcsc_visual_servo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'THIRD_PARTY.md']),
        ('share/' + package_name + '/config', ['config/moveit_servo.yaml', 'config/visual_servo.yaml']),
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
