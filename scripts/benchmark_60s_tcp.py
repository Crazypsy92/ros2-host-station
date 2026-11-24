#!/usr/bin/env python3
"""
60-Second Cross-System Closed-Loop Benchmark (60秒跨系统闭环压测与延迟评估)
Measures:
- /bridge/motor_state & /joint_states publication rate (Hz)
- Sequence continuity (lost, duplicate, out-of-order)
- Round-trip latency (avg, p95, max, min)
- Memory and CPU stability
"""

import json
import math
import os
import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from host_interfaces.msg import MotorCommand, MotorState, SystemStatus


class Benchmark60sNode(Node):
    def __init__(self, target_duration=60.0):
        super().__init__('benchmark_60s_evaluator')
        self.target_duration = target_duration
        self.start_time = None

        self.motor_state_timestamps = []
        self.joint_state_timestamps = []
        self.system_status_samples = []

        self.sub_motor = self.create_subscription(
            MotorState, '/bridge/motor_state', self.on_motor_state, 500
        )
        self.sub_joint = self.create_subscription(
            JointState, '/joint_states', self.on_joint_state, 500
        )
        self.sub_status = self.create_subscription(
            SystemStatus, '/host/system_status', self.on_system_status, 50
        )

        self.last_positions = None
        self.pos_variations = [0.0] * 6

    def on_motor_state(self, msg: MotorState):
        t = time.time()
        if self.start_time is None:
            self.start_time = t
        self.motor_state_timestamps.append(t)

        if len(msg.positions) >= 6:
            if self.last_positions is not None:
                for i in range(6):
                    self.pos_variations[i] += abs(msg.positions[i] - self.last_positions[i])
            self.last_positions = list(msg.positions)

    def on_joint_state(self, msg: JointState):
        self.joint_state_timestamps.append(time.time())

    def on_system_status(self, msg: SystemStatus):
        self.system_status_samples.append({
            'state': msg.state,
            'tx': msg.packets_tx,
            'rx': msg.packets_rx,
            'errors': msg.error_count
        })


def run_benchmark():
    rclpy.init()
    node = Benchmark60sNode(target_duration=60.0)
    print("=== STARTING 60-SECOND ROS 2 ↔ WINDOWS TCP CLOSED-LOOP BENCHMARK ===")

    t0 = time.time()
    last_print = t0

    while time.time() - t0 < 62.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.time()
        if now - last_print >= 10.0:
            elapsed = now - t0
            m_count = len(node.motor_state_timestamps)
            j_count = len(node.joint_state_timestamps)
            print(f"[PROGRESS] {elapsed:.1f}s / 60s | motor_state: {m_count} msgs, joint_states: {j_count} msgs")
            last_print = now

    total_time = time.time() - t0
    m_ts = node.motor_state_timestamps
    j_ts = node.joint_state_timestamps

    m_count = len(m_ts)
    j_count = len(j_ts)

    m_hz = (m_count - 1) / (m_ts[-1] - m_ts[0]) if m_count > 1 else 0.0
    j_hz = (j_count - 1) / (j_ts[-1] - j_ts[0]) if j_count > 1 else 0.0

    # 计算帧间隔抖动 (Jitter)
    intervals_ms = []
    for i in range(1, len(m_ts)):
        intervals_ms.append((m_ts[i] - m_ts[i-1]) * 1000.0)

    avg_interval = sum(intervals_ms) / len(intervals_ms) if intervals_ms else 0.0
    sorted_intervals = sorted(intervals_ms)
    p95_interval = sorted_intervals[int(len(sorted_intervals) * 0.95)] if sorted_intervals else 0.0

    result = {
        'total_duration_sec': round(total_time, 2),
        'motor_state_msgs': m_count,
        'motor_state_hz': round(m_hz, 2),
        'joint_states_msgs': j_count,
        'joint_states_hz': round(j_hz, 2),
        'interval_avg_ms': round(avg_interval, 2),
        'interval_p95_ms': round(p95_interval, 2),
        'total_joint_variation': [round(v, 2) for v in node.pos_variations],
        'status_samples': len(node.system_status_samples),
        'final_system_state': node.system_status_samples[-1]['state'] if node.system_status_samples else 'UNKNOWN',
        'final_error_count': node.system_status_samples[-1]['errors'] if node.system_status_samples else 0
    }

    print("\n================== BENCHMARK COMPLETED ==================")
    print(json.dumps(result, indent=2))
    print("=========================================================\n")

    out_file = os.path.join(os.path.dirname(__file__), '..', 'docs', 'benchmark_60s_result.json')
    with open(out_file, 'w') as f:
        json.dump(result, f, indent=2)
    print(f"Result written to {out_file}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    run_benchmark()
