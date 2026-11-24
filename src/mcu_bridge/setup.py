from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mcu_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Bridge',
    maintainer_email='bridge@local.dev',
    description='ROS 2 MCU Bridge Node and Hardware Loopback Test',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mcu_bridge_node = mcu_bridge.mcu_serial_node:main',
            'mcu_virtual_device = mcu_bridge.mcu_virtual_device:main',
        ],
    },
)
