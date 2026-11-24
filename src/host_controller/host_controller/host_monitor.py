#!/usr/bin/env python3
"""
Host Monitor CLI Dashboard (上位机实时终端监测看板)
"""

import sys
import os
import rclpy
from rclpy.node import Node
from host_interfaces.msg import SystemStatus, MotorState, MotorCommand


class HostMonitor(Node):
    def __init__(self):
        super().__init__('host_monitor')

        self.status = SystemStatus()
        self.cmd = MotorCommand()
        self.state = MotorState()
        self.last_update = self.get_clock().now()

        self.sub_status = self.create_subscription(
            SystemStatus, '/host/system_status', self.on_status, 10
        )
        self.sub_cmd = self.create_subscription(
            MotorCommand, '/host/motor_command', self.on_cmd, 10
        )
        self.sub_state = self.create_subscription(
            MotorState, '/bridge/motor_state', self.on_state, 10
        )

        # 终端刷新定时器 (10 Hz)
        self.timer = self.create_timer(0.1, self.render_screen)

    def on_status(self, msg: SystemStatus):
        self.status = msg

    def on_cmd(self, msg: MotorCommand):
        self.cmd = msg

    def on_state(self, msg: MotorState):
        self.state = msg

    def render_screen(self):
        # ANSI Escape Codes for nice styling
        CLEAR = "\033[H\033[J"
        BOLD = "\033[1m"
        RESET = "\033[0m"
        CYAN = "\033[36m"
        GREEN = "\033[32m"
        YELLOW = "\033[33m"
        RED = "\033[31m"
        MAGENTA = "\033[35m"

        state_color = GREEN if self.status.state in ['READY', 'RUNNING'] else (RED if 'STOP' in self.status.state else YELLOW)

        out = []
        out.append(f"{CLEAR}{BOLD}================================================================================{RESET}")
        out.append(f" {CYAN}ROS 2 上位机综合遥测看板 (Host Supervisory Terminal Dashboard){RESET}")
        out.append(f"{BOLD}================================================================================{RESET}")
        out.append(f" 系统状态 : {state_color}{BOLD}{self.status.state or 'CONNECTING'}{RESET}  |  急停开关: {RED if self.status.emergency_stop else GREEN}{'触发 (ACTIVE)' if self.status.emergency_stop else '正常 (NORMAL)'}{RESET}  |  后端: {MAGENTA}{self.status.active_backend or 'N/A'}{RESET}")
        out.append(f" 发送包数 : {self.status.packets_tx:<10} 接收包数 : {self.status.packets_rx:<10} 错误数: {RED if self.status.error_count > 0 else GREEN}{self.status.error_count}{RESET}  频率: {self.status.loop_frequency:.1f}Hz")
        out.append(f" 状态消息 : {YELLOW}{self.status.status_message or 'None'}{RESET}")
        out.append(f"--------------------------------------------------------------------------------")
        out.append(f"{BOLD}{'关节名称':<12} {'控制模式':<10} {'目标位置(rad)':<16} {'实际位置(rad)':<16} {'实际速度':<12} {'温度(℃)':<8}{RESET}")
        out.append(f"--------------------------------------------------------------------------------")

        names = self.state.joint_names or self.cmd.joint_names or [f"joint_{i+1}" for i in range(6)]
        for i, name in enumerate(names):
            target_pos = self.cmd.positions[i] if i < len(self.cmd.positions) else 0.0
            cur_pos = self.state.positions[i] if i < len(self.state.positions) else 0.0
            cur_vel = self.state.velocities[i] if i < len(self.state.velocities) else 0.0
            temp = self.state.temperatures[i] if i < len(self.state.temperatures) else 25.0

            mode_str = {1: "POS", 2: "VEL", 3: "EFF", 4: "DIS"}.get(self.cmd.control_mode, "IDLE")

            # 误差着色
            err = abs(target_pos - cur_pos)
            err_color = GREEN if err < 0.05 else (YELLOW if err < 0.2 else RED)

            out.append(f" {name:<11} {mode_str:<10} {target_pos:>14.3f}  {err_color}{cur_pos:>14.3f}{RESET}  {cur_vel:>10.2f}   {temp:>6.1f}")

        out.append(f"--------------------------------------------------------------------------------")
        out.append(f" 快捷指令提示: 可以另开终端运行 {CYAN}ros2 run host_controller host_teleop{RESET} 产生轨迹测试")
        out.append(f" 急停触发测试: {RED}ros2 service call /host/emergency_stop host_interfaces/srv/EmergencyStop '{{stop: true, reason: \"Test\"}}'{RESET}")
        out.append(f" 按 Ctrl+C 退出看板")

        sys.stdout.write("\n".join(out) + "\n")
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = HostMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
