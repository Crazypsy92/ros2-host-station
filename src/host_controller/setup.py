from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'host_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bridge',
    maintainer_email='bridge@local.dev',
    description='ROS 2 Host Supervisory Controller and Dashboard',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'host_manager = host_controller.host_manager:main',
            'host_monitor = host_controller.host_monitor:main',
            'host_teleop = host_controller.host_teleop:main',
        ],
    },
)
