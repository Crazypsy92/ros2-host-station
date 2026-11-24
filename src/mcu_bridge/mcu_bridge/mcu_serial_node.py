#!/usr/bin/env python3
"""
MCU Communication Bridge Node (单片机串口/网络通信桥接节点)
"""

import socket
import threading
import time
import rclpy
from rclpy.node import Node
from host_interfaces.msg import MotorCommand, MotorState
from .protocol import (
    pack_motor_command,
    unpack_motor_feedback,
    parse_stream,
    MSG_FEEDBACK_MOTOR
)

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


class McuBridgeNode(Node):
    def __init__(self):
        super().__init__('mcu_bridge_node')

        self.declare_parameter('transport', 'udp')        # 'serial' 或 'udp'
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('udp_target_ip', '127.0.0.1')
        self.declare_parameter('udp_target_port', 9876)
        self.declare_parameter('udp_local_port', 9877)

        self.transport = self.get_parameter('transport').value
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.udp_target_ip = self.get_parameter('udp_target_ip').value
        self.udp_target_port = self.get_parameter('udp_target_port').value
        self.udp_local_port = self.get_parameter('udp_local_port').value

        self.joint_names = [f'joint_{i+1}' for i in range(6)]
        self.running = True
        self.rx_buffer = bytearray()
        self.tx_lock = threading.Lock()

        # 通信句柄
        self.ser = None
        self.udp_sock = None

        # 发布者与订阅者
        self.pub_motor_state = self.create_publisher(MotorState, '/bridge/motor_state', 10)
        self.sub_motor_cmd = self.create_subscription(
            MotorCommand, '/host/motor_command', self.on_motor_command, 10
        )

        self.init_transport()

        # 启动接收线程
        self.rx_thread = threading.Thread(target=self.rx_worker, daemon=True)
        self.rx_thread.start()

        self.get_logger().info(f'MCU Bridge Node running over [{self.transport.upper()}]')

    def init_transport(self):
        if self.transport == 'serial':
            if not SERIAL_AVAILABLE:
                self.get_logger().error('pyserial not installed! Run: pip install pyserial')
                return
            try:
                self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.02)
                self.get_logger().info(f'Opened serial port {self.serial_port} @ {self.baudrate}')
            except Exception as e:
                self.get_logger().error(f'Failed to open serial port {self.serial_port}: {e}')
        else:  # UDP
            try:
                self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.udp_sock.bind(('0.0.0.0', self.udp_local_port))
                self.udp_sock.settimeout(0.05)
                self.get_logger().info(f'UDP socket bound to port {self.udp_local_port}, target {self.udp_target_ip}:{self.udp_target_port}')
            except Exception as e:
                self.get_logger().error(f'Failed to bind UDP socket: {e}')

    def on_motor_command(self, msg: MotorCommand):
        # 编码数据帧并发送给单片机
        frame = pack_motor_command(msg.control_mode, msg.positions)
        with self.tx_lock:
            try:
                if self.transport == 'serial' and self.ser and self.ser.is_open:
                    self.ser.write(frame)
                elif self.transport == 'udp' and self.udp_sock:
                    self.udp_sock.sendto(frame, (self.udp_target_ip, self.udp_target_port))
            except Exception as e:
                self.get_logger().warn(f'Send error: {e}')

    def rx_worker(self):
        while self.running and rclpy.ok():
            try:
                new_data = b''
                if self.transport == 'serial' and self.ser and self.ser.is_open:
                    if self.ser.in_waiting > 0:
                        new_data = self.ser.read(self.ser.in_waiting)
                    else:
                        time.sleep(0.002)
                elif self.transport == 'udp' and self.udp_sock:
                    try:
                        data, _ = self.udp_sock.recvfrom(2048)
                        new_data = data
                    except socket.timeout:
                        continue

                if new_data:
                    self.rx_buffer.extend(new_data)
                    frames = parse_stream(self.rx_buffer)
                    for msg_id, payload in frames:
                        if msg_id == MSG_FEEDBACK_MOTOR:
                            res = unpack_motor_feedback(payload)
                            if res:
                                pos, vel, temps, fault = res
                                state_msg = MotorState()
                                state_msg.header.stamp = self.get_clock().now().to_msg()
                                state_msg.header.frame_id = 'base_link'
                                state_msg.joint_names = self.joint_names
                                state_msg.positions = pos
                                state_msg.velocities = vel
                                state_msg.temperatures = temps
                                state_msg.fault_codes = [fault] + [0] * (len(self.joint_names) - 1)
                                self.pub_motor_state.publish(state_msg)
            except Exception as e:
                self.get_logger().warn(f'RX worker exception: {e}')
                time.sleep(0.05)

    def destroy_node(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        if self.udp_sock:
            self.udp_sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = McuBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
