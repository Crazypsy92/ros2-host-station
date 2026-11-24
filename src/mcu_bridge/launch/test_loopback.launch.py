#!/usr/bin/env python3
"""
MCU Communication Loopback Test Launch
Launches:
1. host_manager (上位机控制节点, backend=MCU_UDP)
2. mcu_bridge_node (上位机通信桥接)
3. mcu_virtual_device (模拟 STM32 单片机固件)
4. host_teleop (生成测试运动指令)
"""

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    host_mgr = Node(
        package='host_controller',
        executable='host_manager',
        name='host_manager',
        output='screen',
        parameters=[{
            'backend': 'MCU_UDP',
            'rate_hz': 50.0,
        }]
    )

    mcu_bridge = Node(
        package='mcu_bridge',
        executable='mcu_bridge_node',
        name='mcu_bridge_node',
        output='screen',
        parameters=[{
            'transport': 'udp',
            'udp_target_ip': '127.0.0.1',
            'udp_target_port': 9876,
            'udp_local_port': 9877,
        }]
    )

    mcu_sim = Node(
        package='mcu_bridge',
        executable='mcu_virtual_device',
        name='mcu_virtual_device',
        output='screen',
        parameters=[{
            'udp_port': 9876,
            'host_ip': '127.0.0.1',
            'host_port': 9877,
            'frequency': 50.0,
        }]
    )

    teleop = Node(
        package='host_controller',
        executable='host_teleop',
        name='host_teleop',
        output='screen',
        parameters=[{
            'frequency': 20.0,
            'amplitude': 0.8,
            'period': 4.0,
            'mode': 'sine',
        }]
    )

    return LaunchDescription([
        host_mgr,
        mcu_bridge,
        mcu_sim,
        teleop,
    ])
