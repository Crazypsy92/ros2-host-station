#!/usr/bin/env python3
"""
Windows TCP Test Server for Cross-System MuJoCo Simulation Link
(Windows 宿主机 TCP 测试服务端与虚拟机械臂执行器)
"""

import argparse
import socket
import sys
import threading
import time
import os

# Import protocol from src/mujoco_bridge/mujoco_bridge/tcp_protocol.py
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'mujoco_bridge'))
from mujoco_bridge.tcp_protocol import (
    StreamParser,
    pack_feedback_frame,
    unpack_command_payload,
    TYPE_CMD_MOTOR,
    TYPE_HEARTBEAT
)


class SimulatedArmActuator:
    """6-DOF Arm Actuator Dynamic Simulator (6轴机械臂执行器动力学仿真)"""
    def __init__(self, num_joints=6, kp=15.0):
        self.num_joints = num_joints
        self.kp = kp
        self.positions = [0.0] * num_joints
        self.velocities = [0.0] * num_joints
        self.efforts = [0.0] * num_joints
        self.temperatures = [26.0 + 0.3 * i for i in range(num_joints)]
        self.target_positions = [0.0] * num_joints
        self.target_velocities = [0.0] * num_joints
        self.mode = 1
        self.fault = 0
        self.lock = threading.Lock()

    def update_command(self, mode, target_pos, target_vel=None):
        with self.lock:
            self.mode = mode
            for i in range(min(self.num_joints, len(target_pos))):
                self.target_positions[i] = target_pos[i]

    def step(self, dt):
        with self.lock:
            for i in range(self.num_joints):
                if self.mode == 4:  # DISABLE
                    self.velocities[i] *= 0.9
                    self.positions[i] += self.velocities[i] * dt
                else:  # POSITION
                    err = self.target_positions[i] - self.positions[i]
                    v_target = max(-5.0, min(5.0, self.kp * err))
                    self.velocities[i] += (v_target - self.velocities[i]) * min(1.0, 10.0 * dt)
                    self.positions[i] += self.velocities[i] * dt
                    self.efforts[i] = self.kp * err
                    self.temperatures[i] = 26.0 + abs(self.velocities[i]) * 1.8

    def get_state(self):
        with self.lock:
            return list(self.positions), list(self.velocities), list(self.efforts), list(self.temperatures), self.fault


class WindowsTcpServer:
    def __init__(self, bind_host='0.0.0.0', port=9088, rate_hz=50.0):
        self.bind_host = bind_host
        self.port = port
        self.rate_hz = rate_hz
        self.running = False
        self.actuator = SimulatedArmActuator()
        self.server_sock = None
        self.last_echo_seq = 0
        self.last_echo_ts = 0.0

    def start(self):
        self.running = True
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind((self.bind_host, self.port))
        self.server_sock.listen(2)
        print(f"[Windows TCP Server] Listening on {self.bind_host}:{self.port} (target {self.rate_hz} Hz)...")

        try:
            while self.running:
                try:
                    self.server_sock.settimeout(1.0)
                    conn, addr = self.server_sock.accept()
                    print(f"[Windows TCP Server] Client connected from {addr}")
                    self.handle_client(conn, addr)
                except socket.timeout:
                    continue
                except Exception as e:
                    if self.running:
                        print(f"[Windows TCP Server] Accept exception: {e}")
        finally:
            self.stop()

    def handle_client(self, conn, addr):
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        parser = StreamParser()
        tx_seq = 0
        dt = 1.0 / self.rate_hz

        client_running = True

        def tx_loop():
            nonlocal tx_seq, client_running
            while self.running and client_running:
                t_start = time.time()
                self.actuator.step(dt)
                pos, vel, eff, temp, fault = self.actuator.get_state()

                tx_seq += 1
                now = time.time()
                feedback_frame = pack_feedback_frame(
                    seq=tx_seq,
                    timestamp=now,
                    echo_seq=self.last_echo_seq,
                    echo_timestamp=self.last_echo_ts,
                    positions=pos,
                    velocities=vel,
                    efforts=eff,
                    temperatures=temp,
                    fault=fault
                )
                try:
                    conn.sendall(feedback_frame)
                except Exception:
                    client_running = False
                    break

                elapsed = time.time() - t_start
                sleep_time = max(0.001, dt - elapsed)
                time.sleep(sleep_time)

        tx_thread = threading.Thread(target=tx_loop, daemon=True)
        tx_thread.start()

        try:
            while self.running and client_running:
                data = conn.recv(4096)
                if not data:
                    print(f"[Windows TCP Server] Client {addr} disconnected gracefully")
                    break

                parser.feed(data)
                while True:
                    frame = parser.parse_next()
                    if not frame:
                        break
                    if frame['type'] == TYPE_CMD_MOTOR:
                        res = unpack_command_payload(frame['payload'])
                        if res:
                            mode, pos, vel, eff = res
                            self.last_echo_seq = frame['seq']
                            self.last_echo_ts = frame['timestamp']
                            self.actuator.update_command(mode, pos, vel)
                    elif frame['type'] == TYPE_HEARTBEAT:
                        self.last_echo_seq = frame['seq']
                        self.last_echo_ts = frame['timestamp']
        except Exception as e:
            print(f"[Windows TCP Server] Client error: {e}")
        finally:
            client_running = False
            try:
                conn.close()
            except Exception:
                pass
            print(f"[Windows TCP Server] Connection closed for {addr}")

    def stop(self):
        self.running = False
        if self.server_sock:
            try:
                self.server_sock.close()
            except Exception:
                pass
            self.server_sock = None
        print("[Windows TCP Server] Stopped.")


def main():
    parser = argparse.ArgumentParser(description="Windows TCP Simulation Server")
    parser.add_argument('--bind', default='0.0.0.0', help="Bind host address")
    parser.add_argument('--port', type=int, default=9088, help="Listening port")
    parser.add_argument('--rate', type=float, default=50.0, help="Feedback frequency in Hz")
    args = parser.parse_args()

    server = WindowsTcpServer(bind_host=args.bind, port=args.port, rate_hz=args.rate)
    try:
        server.start()
    except KeyboardInterrupt:
        print("\nShutdown requested by user.")
    finally:
        server.stop()


if __name__ == '__main__':
    main()
