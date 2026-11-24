#!/usr/bin/env python3
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    rate_arg = DeclareLaunchArgument('rate_hz', default_value='50.0')

    # 1. 上位机主控
    host_mgr = Node(
        package='host_controller',
        executable='host_manager',
        name='host_manager',
        output='screen',
        parameters=[{
            'backend': 'MUJOCO',
            'rate_hz': LaunchConfiguration('rate_hz'),
        }]
    )

    # 2. MuJoCo 仿真桥接节点
    mujoco_bridge = Node(
        package='mujoco_bridge',
        executable='mujoco_bridge_node',
        name='mujoco_bridge_node',
        output='screen',
        parameters=[{
            'rate_hz': LaunchConfiguration('rate_hz'),
            'num_joints': 6,
        }]
    )

    # 3. 仿真轨迹发生器 (生成平滑测试正弦轨迹)
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
        rate_arg,
        host_mgr,
        mujoco_bridge,
        teleop,
    ])
