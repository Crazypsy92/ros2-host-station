#!/usr/bin/env python3
"""
Virtual MCU Device Simulator (单片机下位机虚拟仿真器)
Simulates STM32 / ESP32 firmware loop: receives commands, executes motor closed-loop, transmits feedback.
"""

import socket
import time
import rclpy
from rclpy.node import Node
from .protocol import (
    pack_motor_feedback,
    unpack_motor_command,
    parse_stream,
    MSG_CMD_MOTOR,
    MSG_CMD_ESTOP
)


class McuVirtualDevice(Node):
    def __init__(self):
        super().__init__('mcu_virtual_device')

        self.declare_parameter('udp_port', 9876)
        self.declare_parameter('host_ip', '127.0.0.1')
        self.declare_parameter('host_port', 9877)
        self.declare_parameter('frequency', 50.0)

        self.udp_port = self.get_parameter('udp_port').value
        self.host_ip = self.get_parameter('host_ip').value
        self.host_port = self.get_parameter('host_port').value
        self.freq = self.get_parameter('frequency').value

        # 单片机内部状态仿真
        self.num_joints = 6
        self.target_positions = [0.0] * self.num_joints
        self.cur_positions = [0.0] * self.num_joints
        self.cur_velocities = [0.0] * self.num_joints
        self.temperatures = [32.0] * self.num_joints
        self.fault_code = 0
        self.e_stop = False

        self.rx_buffer = bytearray()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.udp_port))
        self.sock.setblocking(False)

        self.dt = 1.0 / self.freq
        self.timer = self.create_timer(self.dt, self.firmware_loop)
        self.get_logger().info(f'Virtual STM32 MCU Simulator active on UDP port {self.udp_port} -> {self.host_ip}:{self.host_port}')

    def firmware_loop(self):
        # 1. 模拟单片机 UART/CAN/Network 中断接收
        try:
            while True:
                data, addr = self.sock.recvfrom(1024)
                if not data:
                    break
                self.rx_buffer.extend(data)
        except BlockingIOError:
            pass

        # 解析帧
        frames = parse_stream(self.rx_buffer)
        for msg_id, payload in frames:
            if msg_id == MSG_CMD_MOTOR:
                mode, pos = unpack_motor_command(payload)
                if not self.e_stop:
                    for i in range(min(self.num_joints, len(pos))):
                        self.target_positions[i] = pos[i]
            elif msg_id == MSG_CMD_ESTOP:
                self.e_stop = True

        # 2. 模拟单片机 FOC / PID 闭环运动控制
        kp = 18.0
        for i in range(self.num_joints):
            err = self.target_positions[i] - self.cur_positions[i]
            # 电机响应
            v = kp * err
            v = max(-6.0, min(6.0, v))
            self.cur_velocities[i] += (v - self.cur_velocities[i]) * min(1.0, 12.0 * self.dt)
            self.cur_positions[i] += self.cur_velocities[i] * self.dt
            self.temperatures[i] = 30.0 + abs(self.cur_velocities[i]) * 1.5

        # 3. 打包并发送反馈帧 (模拟单片机周期性上传)
        feedback_frame = pack_motor_feedback(
            self.cur_positions,
            self.cur_velocities,
            self.temperatures,
            self.fault_code
        )
        try:
            self.sock.sendto(feedback_frame, (self.host_ip, self.host_port))
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = McuVirtualDevice()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
