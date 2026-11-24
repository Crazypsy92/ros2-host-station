#!/usr/bin/env python3
"""
MuJoCo Simulation Bridge Node (MuJoCo 物理引擎与机械臂仿真桥接节点)
"""

import math
import time
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from host_interfaces.msg import MotorCommand, MotorState

# Try importing mujoco if available
try:
    import mujoco
    MUJOCO_AVAILABLE = True
except ImportError:
    MUJOCO_AVAILABLE = False


class MujocoBridgeNode(Node):
    def __init__(self):
        super().__init__('mujoco_bridge_node')

        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('num_joints', 6)
        self.declare_parameter('model_path', '')
        self.declare_parameter('kp', 15.0)  # 仿真电机位置响应刚度
        self.declare_parameter('kd', 1.0)   # 仿真阻尼

        self.rate_hz = self.get_parameter('rate_hz').value
        self.num_joints = self.get_parameter('num_joints').value
        self.model_path = self.get_parameter('model_path').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value

        self.joint_names = [f'joint_{i+1}' for i in range(self.num_joints)]

        # 内部状态变量
        self.positions = [0.0] * self.num_joints
        self.velocities = [0.0] * self.num_joints
        self.efforts = [0.0] * self.num_joints
        self.temperatures = [28.5 + 0.2 * i for i in range(self.num_joints)]

        # 目标指令缓存
        self.target_positions = [0.0] * self.num_joints
        self.target_velocities = [0.0] * self.num_joints
        self.target_efforts = [0.0] * self.num_joints
        self.control_mode = MotorCommand.MODE_POSITION

        # 发布者
        self.pub_motor_state = self.create_publisher(MotorState, '/bridge/motor_state', 10)
        self.pub_joint_state = self.create_publisher(JointState, '/joint_states', 10)

        # 订阅者 (来自上位机控制命令)
        self.sub_cmd = self.create_subscription(
            MotorCommand, '/host/motor_command', self.on_motor_command, 10
        )

        # 初始化 MuJoCo 物理引擎或备用动力学仿真器
        self.mj_model = None
        self.mj_data = None
        self.init_simulation()

        # 仿真主循环定时器
        self.dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(self.dt, self.step_simulation)
        self.get_logger().info(f'MuJoCo Bridge running at {self.rate_hz} Hz ({self.num_joints} joints)')

    def init_simulation(self):
        if MUJOCO_AVAILABLE and self.model_path and os.path.exists(self.model_path):
            try:
                self.mj_model = mujoco.MjModel.from_xml_path(self.model_path)
                self.mj_data = mujoco.MjData(self.mj_model)
                self.get_logger().info(f'Successfully loaded MuJoCo model: {self.model_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to load MuJoCo XML: {e}, falling back to virtual dynamics')
        else:
            status = 'MuJoCo library not loaded' if not MUJOCO_AVAILABLE else 'No model XML specified'
            self.get_logger().info(f'{status} -> Using high-fidelity virtual robotic arm simulator')

    def on_motor_command(self, msg: MotorCommand):
        self.control_mode = msg.control_mode
        for i in range(min(self.num_joints, len(msg.positions))):
            self.target_positions[i] = msg.positions[i]
        for i in range(min(self.num_joints, len(msg.velocities))):
            self.target_velocities[i] = msg.velocities[i]
        for i in range(min(self.num_joints, len(msg.efforts))):
            self.target_efforts[i] = msg.efforts[i]

    def step_simulation(self):
        now_msg = self.get_clock().now().to_msg()

        # 1. 物理步进计算
        if self.mj_model and self.mj_data:
            # 填入控制量并步进 MuJoCo
            for i in range(min(len(self.mj_data.ctrl), self.num_joints)):
                self.mj_data.ctrl[i] = self.target_positions[i]
            mujoco.mj_step(self.mj_model, self.mj_data)

            # 读取状态
            for i in range(self.num_joints):
                if i < len(self.mj_data.qpos):
                    self.positions[i] = float(self.mj_data.qpos[i])
                    self.velocities[i] = float(self.mj_data.qvel[i])
        else:
            # 内建高保真动力学一阶/二阶响应 (闭环测试)
            if self.control_mode == MotorCommand.MODE_DISABLE:
                # 掉电/松闸：速度衰减
                for i in range(self.num_joints):
                    self.velocities[i] *= 0.95
                    self.positions[i] += self.velocities[i] * self.dt
            elif self.control_mode == MotorCommand.MODE_VELOCITY:
                for i in range(self.num_joints):
                    self.velocities[i] = self.target_velocities[i]
                    self.positions[i] += self.velocities[i] * self.dt
            else:  # MODE_POSITION
                for i in range(self.num_joints):
                    pos_err = self.target_positions[i] - self.positions[i]
                    # PD 响应
                    target_vel = self.kp * pos_err
                    # 速度限幅
                    target_vel = max(-5.0, min(5.0, target_vel))
                    self.velocities[i] += (target_vel - self.velocities[i]) * min(1.0, 10.0 * self.dt)
                    self.positions[i] += self.velocities[i] * self.dt
                    self.efforts[i] = self.kp * pos_err

        # 2. 发布上位机专用的 MotorState
        motor_msg = MotorState()
        motor_msg.header.stamp = now_msg
        motor_msg.header.frame_id = 'base_link'
        motor_msg.joint_names = self.joint_names
        motor_msg.positions = [float(p) for p in self.positions]
        motor_msg.velocities = [float(v) for v in self.velocities]
        motor_msg.efforts = [float(e) for e in self.efforts]
        motor_msg.temperatures = [float(t + 0.05 * abs(v)) for t, v in zip(self.temperatures, self.velocities)]
        motor_msg.fault_codes = [0] * self.num_joints
        self.pub_motor_state.publish(motor_msg)

        # 3. 发布标准 ROS 2 JointState (支持 RViz 可视化)
        js = JointState()
        js.header.stamp = now_msg
        js.name = self.joint_names
        js.position = [float(p) for p in self.positions]
        js.velocity = [float(v) for v in self.velocities]
        js.effort = [float(e) for e in self.efforts]
        self.pub_joint_state.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = MujocoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
