#!/usr/bin/env python3
"""
Windows TCP Cross-System Bridge Launch File
Launches:
1. host_manager (上位机控制节点, backend=WINDOWS_TCP)
2. windows_tcp_bridge_node (跨系统 TCP 通信桥接客户端)
3. host_teleop (运动轨迹测试发生器)
"""

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host_ip_arg = DeclareLaunchArgument(
        'host_ip',
        default_value='auto',
        description='Windows host IP address (auto detects WSL default gateway)'
    )
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='9088',
        description='TCP port of Windows test server'
    )
    rate_arg = DeclareLaunchArgument(
        'rate_hz',
        default_value='50.0',
        description='Control and state frequency in Hz'
    )

    host_mgr = Node(
        package='host_controller',
        executable='host_manager',
        name='host_manager',
        output='screen',
        parameters=[{
            'backend': 'WINDOWS_TCP',
            'rate_hz': LaunchConfiguration('rate_hz'),
        }]
    )

    tcp_bridge = Node(
        package='mujoco_bridge',
        executable='windows_tcp_bridge_node',
        name='windows_tcp_bridge_node',
        output='screen',
        parameters=[{
            'host_ip': LaunchConfiguration('host_ip'),
            'port': LaunchConfiguration('port'),
            'reconnect_interval': 1.0,
            'num_joints': 6,
        }]
    )

    teleop = Node(
        package='host_controller',
        executable='host_teleop',
        name='host_teleop',
        output='screen',
        parameters=[{
            'frequency': 20.0,
            'amplitude': 0.6,
            'period': 5.0,
            'mode': 'sine',
        }]
    )

    return LaunchDescription([
        host_ip_arg,
        port_arg,
        rate_arg,
        host_mgr,
        tcp_bridge,
        teleop,
    ])
