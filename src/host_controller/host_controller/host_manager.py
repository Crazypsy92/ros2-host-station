#!/usr/bin/env python3
"""
Host Manager Node (上位机主控状态机与安全仲裁节点)
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from host_interfaces.msg import SystemStatus, MotorCommand, MotorState
from host_interfaces.srv import EmergencyStop, SetControlMode


class HostManager(Node):
    def __init__(self):
        super().__init__('host_manager')

        # 声明参数
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('backend', 'MUJOCO')  # MUJOCO, MCU_SERIAL, MCU_UDP, SIM
        self.declare_parameter('watchdog_timeout', 1.5)

        self.rate_hz = self.get_parameter('rate_hz').value
        self.backend = self.get_parameter('backend').value
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').value

        # 状态机变量
        self.state = 'READY'
        self.emergency_stop = False
        self.e_stop_reason = ''
        self.control_mode = MotorCommand.MODE_POSITION
        self.packets_tx = 0
        self.packets_rx = 0
        self.error_count = 0
        self.last_state_rx_time = self.get_clock().now().nanoseconds * 1e-9
        self.status_message = 'Host station initialized and ready'

        # 当前指令缓存
        self.cached_command = MotorCommand()
        self.cached_command.control_mode = self.control_mode
        self.cached_command.joint_names = [f'joint_{i+1}' for i in range(6)]
        self.cached_command.positions = [0.0] * 6
        self.cached_command.velocities = [0.0] * 6
        self.cached_command.efforts = [0.0] * 6

        self.cb_group = ReentrantCallbackGroup()

        # 发布者
        self.pub_status = self.create_publisher(SystemStatus, '/host/system_status', 10)
        self.pub_command = self.create_publisher(MotorCommand, '/host/motor_command', 10)

        # 订阅者
        self.sub_teleop = self.create_subscription(
            MotorCommand, '/teleop/motor_command', self.on_teleop_command, 10, callback_group=self.cb_group
        )
        self.sub_motor_state = self.create_subscription(
            MotorState, '/bridge/motor_state', self.on_motor_state, 10, callback_group=self.cb_group
        )

        # 服务
        self.srv_estop = self.create_service(
            EmergencyStop, '/host/emergency_stop', self.handle_emergency_stop, callback_group=self.cb_group
        )
        self.srv_set_mode = self.create_service(
            SetControlMode, '/host/set_control_mode', self.handle_set_mode, callback_group=self.cb_group
        )

        # 主控制与状态定时器 (例如 50Hz)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.control_loop)
        self.get_logger().info(f'Host Manager started in {self.backend} mode at {self.rate_hz} Hz')

    def handle_emergency_stop(self, request, response):
        if request.stop:
            self.emergency_stop = True
            self.state = 'EMERGENCY_STOP'
            self.e_stop_reason = request.reason or 'User requested emergency stop'
            self.status_message = f'EMERGENCY STOP: {self.e_stop_reason}'
            self.get_logger().error(f'[E-STOP TRIGGERED] {self.e_stop_reason}')
            response.success = True
            response.message = 'System in EMERGENCY_STOP'
        else:
            self.emergency_stop = False
            self.state = 'READY'
            self.status_message = 'Emergency stop released, system READY'
            self.get_logger().info('[E-STOP RELEASED] System restored to READY')
            response.success = True
            response.message = 'System restored to READY'
        return response

    def handle_set_mode(self, request, response):
        if self.emergency_stop:
            response.success = False
            response.message = 'Cannot change mode during EMERGENCY_STOP'
            return response

        valid_modes = [
            MotorCommand.MODE_POSITION,
            MotorCommand.MODE_VELOCITY,
            MotorCommand.MODE_EFFORT,
            MotorCommand.MODE_DISABLE,
        ]
        if request.mode in valid_modes:
            self.control_mode = request.mode
            self.cached_command.control_mode = self.control_mode
            response.success = True
            response.message = f'Mode switched to {self.control_mode}'
            self.get_logger().info(f'Control mode changed to {self.control_mode}')
        else:
            response.success = False
            response.message = f'Invalid mode {request.mode}'
        return response

    def on_teleop_command(self, msg: MotorCommand):
        if self.emergency_stop:
            return  # 急停状态丢弃外界控制输入

        self.cached_command.joint_names = msg.joint_names
        self.cached_command.positions = msg.positions
        self.cached_command.velocities = msg.velocities
        self.cached_command.efforts = msg.efforts
        if msg.control_mode != 0:
            self.control_mode = msg.control_mode
            self.cached_command.control_mode = self.control_mode
        if self.state == 'READY':
            self.state = 'RUNNING'

    def on_motor_state(self, msg: MotorState):
        self.packets_rx += 1
        self.last_state_rx_time = self.get_clock().now().nanoseconds * 1e-9
        # 检查下位机是否有故障码
        faults = [f for f in msg.fault_codes if f != 0]
        if faults:
            self.error_count += len(faults)
            self.status_message = f'Hardware fault detected: {faults}'

    def control_loop(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt_since_rx = now - self.last_state_rx_time

        # Watchdog 安全机制
        if dt_since_rx > self.watchdog_timeout:
            if not self.emergency_stop:
                self.state = 'WARN_DISCONNECTED'
                self.status_message = f'Watchdog timeout! No telemetry for {dt_since_rx:.1f}s'

        # 发布电机控制命令
        cmd = MotorCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        if self.emergency_stop:
            cmd.control_mode = MotorCommand.MODE_DISABLE
            cmd.joint_names = self.cached_command.joint_names
            cmd.positions = [0.0] * len(self.cached_command.positions)
            cmd.velocities = [0.0] * len(self.cached_command.velocities)
            cmd.efforts = [0.0] * len(self.cached_command.efforts)
        else:
            cmd.control_mode = self.control_mode
            cmd.joint_names = self.cached_command.joint_names
            cmd.positions = self.cached_command.positions
            cmd.velocities = self.cached_command.velocities
            cmd.efforts = self.cached_command.efforts

        self.pub_command.publish(cmd)
        self.packets_tx += 1

        # 发布上位机状态
        status = SystemStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.state = self.state
        status.emergency_stop = self.emergency_stop
        status.loop_frequency = float(self.rate_hz)
        status.packets_tx = self.packets_tx
        status.packets_rx = self.packets_rx
        status.error_count = self.error_count
        status.active_backend = self.backend
        status.status_message = self.status_message
        self.pub_status.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = HostManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
