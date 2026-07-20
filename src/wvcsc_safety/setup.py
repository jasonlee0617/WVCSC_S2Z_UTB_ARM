import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'wvcsc_safety'

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
    description='Fail-closed Nav2 velocity gate and controlled arm recovery.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'safety_gate = wvcsc_safety.safety_node:main',
    ]},
)
