#!/usr/bin/env python3
"""
Host Teleop & Trajectory Generator (上位机测试遥控与正弦激励发生器)
"""

import math
import time
import rclpy
from rclpy.node import Node
from host_interfaces.msg import MotorCommand


class HostTeleop(Node):
    def __init__(self):
        super().__init__('host_teleop')

        self.declare_parameter('frequency', 20.0)
        self.declare_parameter('amplitude', 0.8)     # 正弦摆动幅度 (rad)
        self.declare_parameter('period', 4.0)        # 摆动周期 (s)
        self.declare_parameter('mode', 'sine')       # 'sine', 'step', 'hold'

        self.freq = self.get_parameter('frequency').value
        self.amp = self.get_parameter('amplitude').value
        self.period = self.get_parameter('period').value
        self.mode = self.get_parameter('mode').value

        self.pub = self.create_publisher(MotorCommand, '/teleop/motor_command', 10)
        self.timer = self.create_timer(1.0 / self.freq, self.publish_teleop)
        self.start_time = time.time()

        self.joint_names = [f'joint_{i+1}' for i in range(6)]
        self.get_logger().info(f'Host Teleop started in [{self.mode}] mode, amp={self.amp} rad, T={self.period}s')

    def publish_teleop(self):
        t = time.time() - self.start_time
        msg = MotorCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.control_mode = MotorCommand.MODE_POSITION
        msg.joint_names = self.joint_names

        positions = []
        velocities = []

        if self.mode == 'sine':
            for i in range(len(self.joint_names)):
                # 为各个关节添加相位差
                phase = i * (math.pi / 4.0)
                pos = self.amp * math.sin(2.0 * math.pi / self.period * t + phase)
                vel = self.amp * (2.0 * math.pi / self.period) * math.cos(2.0 * math.pi / self.period * t + phase)
                positions.append(pos)
                velocities.append(vel)
        elif self.mode == 'step':
            step_val = self.amp if int(t / (self.period / 2.0)) % 2 == 0 else -self.amp
            positions = [step_val] * len(self.joint_names)
            velocities = [0.0] * len(self.joint_names)
        else:  # hold
            positions = [0.0] * len(self.joint_names)
            velocities = [0.0] * len(self.joint_names)

        msg.positions = positions
        msg.velocities = velocities
        msg.efforts = [0.0] * len(self.joint_names)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HostTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
