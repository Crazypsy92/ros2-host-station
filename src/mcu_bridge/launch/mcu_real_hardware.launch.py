#!/usr/bin/env python3
"""
MCU Real Hardware Launch (用于真实 STM32/ESP32 硬件串口通信)
"""

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port device path (e.g. /dev/ttyUSB0 or /dev/ttyACM0)'
    )
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baudrate for MCU serial communication'
    )

    host_mgr = Node(
        package='host_controller',
        executable='host_manager',
        name='host_manager',
        output='screen',
        parameters=[{
            'backend': 'MCU_SERIAL',
            'rate_hz': 50.0,
        }]
    )

    mcu_bridge = Node(
        package='mcu_bridge',
        executable='mcu_bridge_node',
        name='mcu_bridge_node',
        output='screen',
        parameters=[{
            'transport': 'serial',
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
        }]
    )

    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        host_mgr,
        mcu_bridge,
    ])
