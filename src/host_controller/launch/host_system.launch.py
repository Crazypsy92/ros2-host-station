#!/usr/bin/env python3
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    backend_arg = DeclareLaunchArgument(
        'backend',
        default_value='MUJOCO',
        description='Active backend: MUJOCO, MCU_SERIAL, MCU_UDP, SIM'
    )
    rate_arg = DeclareLaunchArgument(
        'rate_hz',
        default_value='50.0',
        description='Control loop frequency in Hz'
    )

    host_manager_node = Node(
        package='host_controller',
        executable='host_manager',
        name='host_manager',
        output='screen',
        parameters=[{
            'backend': LaunchConfiguration('backend'),
            'rate_hz': LaunchConfiguration('rate_hz'),
            'watchdog_timeout': 1.5,
        }]
    )

    return LaunchDescription([
        backend_arg,
        rate_arg,
        host_manager_node,
    ])
