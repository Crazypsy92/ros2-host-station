#!/usr/bin/env python3
"""
Full Station Launch File (上位机综合启动入口)
Supports:
  ros2 launch host_controller full_station.launch.py backend:=MUJOCO
  ros2 launch host_controller full_station.launch.py backend:=MCU_LOOPBACK
  ros2 launch host_controller full_station.launch.py backend:=MCU_SERIAL serial_port:=/dev/ttyUSB0
"""

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    backend_arg = DeclareLaunchArgument(
        'backend',
        default_value='MUJOCO',
        description='Backend target: MUJOCO, MCU_LOOPBACK, or MCU_SERIAL'
    )
    rate_arg = DeclareLaunchArgument(
        'rate_hz',
        default_value='50.0',
        description='Control loop frequency in Hz'
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for MCU_SERIAL'
    )
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baudrate for MCU_SERIAL'
    )

    # 1. 主控节点
    host_mgr = Node(
        package='host_controller',
        executable='host_manager',
        name='host_manager',
        output='screen',
        parameters=[{
            'backend': LaunchConfiguration('backend'),
            'rate_hz': LaunchConfiguration('rate_hz'),
        }]
    )

    # 2. 条件启动: MUJOCO 仿真桥接
    mujoco_node = Node(
        package='mujoco_bridge',
        executable='mujoco_bridge_node',
        name='mujoco_bridge_node',
        output='screen',
        condition=LaunchConfigurationEquals('backend', 'MUJOCO'),
        parameters=[{
            'rate_hz': LaunchConfiguration('rate_hz'),
            'num_joints': 6,
        }]
    )

    # 3. 条件启动: MCU 虚拟回环仿真测试
    mcu_sim_node = Node(
        package='mcu_bridge',
        executable='mcu_virtual_device',
        name='mcu_virtual_device',
        output='screen',
        condition=LaunchConfigurationEquals('backend', 'MCU_LOOPBACK'),
        parameters=[{
            'udp_port': 9876,
            'host_ip': '127.0.0.1',
            'host_port': 9877,
            'frequency': 50.0,
        }]
    )
    mcu_bridge_udp = Node(
        package='mcu_bridge',
        executable='mcu_bridge_node',
        name='mcu_bridge_node',
        output='screen',
        condition=LaunchConfigurationEquals('backend', 'MCU_LOOPBACK'),
        parameters=[{
            'transport': 'udp',
            'udp_target_ip': '127.0.0.1',
            'udp_target_port': 9876,
            'udp_local_port': 9877,
        }]
    )

    # 4. 条件启动: MCU 真实硬件串口
    mcu_bridge_serial = Node(
        package='mcu_bridge',
        executable='mcu_bridge_node',
        name='mcu_bridge_node',
        output='screen',
        condition=LaunchConfigurationEquals('backend', 'MCU_SERIAL'),
        parameters=[{
            'transport': 'serial',
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
        }]
    )

    # 5. 默认伴随启动轨迹测试发生器
    teleop_node = Node(
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
        backend_arg,
        rate_arg,
        serial_port_arg,
        baudrate_arg,
        host_mgr,
        mujoco_node,
        mcu_sim_node,
        mcu_bridge_udp,
        mcu_bridge_serial,
        teleop_node,
    ])
