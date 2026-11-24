#!/usr/bin/env python3
"""
WSL2 ROS 2 ↔ Windows TCP Bridge Node
Connects to Windows TCP Server, relays /host/motor_command to Windows,
receives state telemetry and publishes /bridge/motor_state & /joint_states.
"""

import math
import os
import socket
import subprocess
import threading
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from host_interfaces.msg import MotorCommand, MotorState
from .tcp_protocol import (
    StreamParser,
    pack_command_frame,
    unpack_feedback_payload,
    get_default_gateway,
    TYPE_FEEDBACK_MOTOR
)


class WindowsTcpBridgeNode(Node):
    def __init__(self):
        super().__init__('windows_tcp_bridge_node')

        self.declare_parameter('host_ip', 'auto')
        self.declare_parameter('port', 9088)
        self.declare_parameter('reconnect_interval', 1.0)
        self.declare_parameter('num_joints', 6)

        param_host = self.get_parameter('host_ip').value
        if param_host in ('', 'auto'):
            self.host_ip = get_default_gateway()
        else:
            self.host_ip = param_host

        self.port = self.get_parameter('port').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.num_joints = self.get_parameter('num_joints').value
        self.joint_names = [f'joint_{i+1}' for i in range(self.num_joints)]

        self.sock = None
        self.connected = False
        self.running = True
        self.tx_lock = threading.Lock()
        self.tx_seq = 0
        self.expected_rx_seq = 1

        # 统计指标
        self.stats_lock = threading.Lock()
        self.total_tx = 0
        self.total_rx = 0
        self.seq_drops = 0
        self.seq_out_of_order = 0
        self.seq_duplicates = 0
        self.latencies_ms = []

        # ROS 2 发布者与订阅者
        self.pub_motor_state = self.create_publisher(MotorState, '/bridge/motor_state', 10)
        self.pub_joint_state = self.create_publisher(JointState, '/joint_states', 10)

        self.sub_cmd = self.create_subscription(
            MotorCommand, '/host/motor_command', self.on_motor_command, 10
        )

        # 启动后台 TCP 管理线程
        self.conn_thread = threading.Thread(target=self.connection_worker, daemon=True)
        self.conn_thread.start()

        # 启动周期性延迟统计日志 (每 10 秒)
        self.stats_timer = self.create_timer(10.0, self.log_latency_stats)

        self.get_logger().info(f'Windows TCP Bridge Node initialized. Target: {self.host_ip}:{self.port}')

    def log_latency_stats(self):
        stats = self.get_latency_stats()
        if stats['count'] > 0:
            self.get_logger().info(
                f"[RTT STATS] Samples: {stats['count']} | Avg RTT: {stats['avg']:.2f}ms | P95: {stats['p95']:.2f}ms | "
                f"Min: {stats['min']:.2f}ms | Max: {stats['max']:.2f}ms | Drops: {stats['drops']} | TX: {stats['total_tx']} | RX: {stats['total_rx']}"
            )

    def connection_worker(self):
        parser = StreamParser()

        while self.running and rclpy.ok():
            if not self.connected:
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.settimeout(2.0)
                    s.connect((self.host_ip, self.port))
                    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    s.settimeout(0.5)

                    with self.tx_lock:
                        self.sock = s
                        self.connected = True
                        self.expected_rx_seq = 1

                    self.get_logger().info(f'Successfully connected to Windows TCP Server @ {self.host_ip}:{self.port}')
                except Exception as e:
                    self.connected = False
                    time.sleep(self.reconnect_interval)
                    continue

            # 已连接状态下的数据接收
            try:
                data = self.sock.recv(4096)
                if not data:
                    self.get_logger().warn('Windows TCP Server closed connection. Reconnecting...')
                    self.close_socket()
                    continue

                parser.feed(data)
                now_wall = time.time()
                now_ros = self.get_clock().now().to_msg()

                while True:
                    frame = parser.parse_next()
                    if not frame:
                        break

                    if frame['type'] == TYPE_FEEDBACK_MOTOR:
                        res = unpack_feedback_payload(frame['payload'])
                        if res:
                            echo_seq, echo_ts, pos, vel, eff, temp, fault = res
                            seq = frame['seq']

                            with self.stats_lock:
                                self.total_rx += 1
                                if seq < self.expected_rx_seq:
                                    self.seq_duplicates += 1
                                elif seq > self.expected_rx_seq:
                                    self.seq_drops += (seq - self.expected_rx_seq)
                                    self.expected_rx_seq = seq + 1
                                else:
                                    self.expected_rx_seq += 1

                                if echo_ts > 0.0:
                                    rtt = (now_wall - echo_ts) * 1000.0
                                    if 0.0 <= rtt < 1000.0:
                                        self.latencies_ms.append(rtt)
                                        if len(self.latencies_ms) > 10000:
                                            self.latencies_ms.pop(0)

                            # 发布 MotorState
                            mstate = MotorState()
                            mstate.header.stamp = now_ros
                            mstate.header.frame_id = 'base_link'
                            mstate.joint_names = self.joint_names
                            mstate.positions = pos
                            mstate.velocities = vel
                            mstate.efforts = eff
                            mstate.temperatures = temp
                            mstate.fault_codes = [fault] + [0] * (self.num_joints - 1)
                            self.pub_motor_state.publish(mstate)

                            # 发布 JointState
                            jstate = JointState()
                            jstate.header.stamp = now_ros
                            jstate.name = self.joint_names
                            jstate.position = pos
                            jstate.velocity = vel
                            jstate.effort = eff
                            self.pub_joint_state.publish(jstate)
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().warn(f'Socket error: {e}. Reconnecting...')
                self.close_socket()
                time.sleep(self.reconnect_interval)

    def on_motor_command(self, msg: MotorCommand):
        if not self.connected or not self.sock:
            return

        with self.tx_lock:
            self.tx_seq += 1
            now = time.time()
            frame = pack_command_frame(
                seq=self.tx_seq,
                timestamp=now,
                mode=msg.control_mode,
                positions=msg.positions,
                velocities=msg.velocities,
                efforts=msg.efforts
            )
            try:
                self.sock.sendall(frame)
                with self.stats_lock:
                    self.total_tx += 1
            except Exception as e:
                self.get_logger().warn(f'Failed to send command over TCP: {e}')
                self.close_socket()

    def get_latency_stats(self):
        with self.stats_lock:
            if not self.latencies_ms:
                return {'count': 0, 'avg': 0.0, 'p95': 0.0, 'max': 0.0, 'drops': self.seq_drops}
            arr = sorted(self.latencies_ms)
            p95_idx = int(len(arr) * 0.95)
            return {
                'count': len(arr),
                'total_tx': self.total_tx,
                'total_rx': self.total_rx,
                'avg': sum(arr) / len(arr),
                'p95': arr[p95_idx] if p95_idx < len(arr) else arr[-1],
                'max': arr[-1],
                'min': arr[0],
                'drops': self.seq_drops,
                'duplicates': self.seq_duplicates,
                'out_of_order': self.seq_out_of_order
            }

    def close_socket(self):
        self.connected = False
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    def destroy_node(self):
        self.running = False
        self.close_socket()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WindowsTcpBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
