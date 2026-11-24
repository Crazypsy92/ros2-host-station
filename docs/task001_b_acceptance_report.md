# Task001-B 验收报告：ROS 2 工作空间构建与基础通信验收

## 1. 基础环境与测试基线

- **操作系统**：Linux NPC05785 6.18.54.1-microsoft-standard-WSL2 (Ubuntu 24.04 LTS x86_64)
- **ROS 2 版本**：ROS 2 Jazzy Jalisco
- **仓库**：`Crazypsy92/ros2-host-station`
- **分支**：`main`
- **测试 Commit**：`c0733b7a86de028a7e1528ab0b88781ac43ad5bb`
- **执行时间**：2026-11-24 18:42:00 (UTC+8)
- **业务代码修改**：**无（0 行修改）**

---

## 2. 工作空间构建与功能包/接口检查

### 2.1 构建命令与结果
```bash
source /opt/ros/jazzy/setup.bash
./manage.sh build
source install/setup.bash
```
**构建日志摘要**：
```text
Starting >>> host_interfaces
Finished <<< host_interfaces [0.91s]
Starting >>> host_controller
Starting >>> mcu_bridge
Starting >>> mujoco_bridge
Finished <<< mujoco_bridge [4.05s]
Finished <<< mcu_bridge [4.08s]
Finished <<< host_controller [4.25s]

Summary: 4 packages finished [5.38s]
```
- **结论**：4 个包全部编译通过，0 警告，0 错误。

### 2.2 功能包发现验证
执行命令：
```bash
ros2 pkg list | grep -E '^(host_interfaces|host_controller|mujoco_bridge|mcu_bridge)$'
```
**输出**：
```text
host_controller
host_interfaces
mcu_bridge
mujoco_bridge
```
4 个功能包均被 ROS 2 准确发现。

### 2.3 自定义消息与服务接口验证
- `ros2 interface show host_interfaces/msg/MotorCommand`：正常显示 `Header`, `uint8 control_mode`, `joint_names`, `positions`, `velocities`, `efforts`。
- `ros2 interface show host_interfaces/srv/EmergencyStop`：正常显示 `bool stop`, `string reason` -> `bool success`, `string message`。

---

## 3. 内部虚拟机械臂闭环运行实测

启动闭环启动文件：
```bash
ros2 launch mujoco_bridge test_bridge.launch.py
```

### 3.1 运行中节点列表
```text
/host_manager
/host_teleop
/mujoco_bridge_node
```
3 个核心节点均已稳定启动。

### 3.2 运行中华题列表
```text
/bridge/motor_state
/host/motor_command
/host/system_status
/joint_states
/parameter_events
/rosout
/teleop/motor_command
```

### 3.3 上位机状态采样 (`/host/system_status`)
```yaml
header:
  stamp:
    sec: 1795516955
    nanosec: 197468100
  frame_id: ''
state: RUNNING
emergency_stop: false
loop_frequency: 50.0
packets_tx: 317
packets_rx: 304
error_count: 0
active_backend: MUJOCO
status_message: Host station initialized and ready
```
系统状态处于预期的 `RUNNING` 模式，软急停未触发，看门狗正常喂狗无超时。

### 3.4 话题实际发布频率实测 (10s 独立采样统计)
- **/bridge/motor_state**：
  - 测量窗口：9.80 秒
  - 接收报文数：500 帧
  - **实测平均频率：50.92 Hz**（配置目标：50.0 Hz）
- **/joint_states**：
  - 测量窗口：9.63 秒
  - 接收报文数：492 帧
  - **实测平均频率：51.00 Hz**（配置目标：50.0 Hz）

### 3.5 六关节位置随指令响应验证
对比时间点 $T_0$ 与 $T_0 + 2\text{s}$ 的 `/joint_states` 实际数据：

| 关节名称 | $T_0$ 位置 (rad) | $T_0+2\text{s}$ 位置 (rad) | 运动状态 |
| :--- | :---: | :---: | :---: |
| **joint_1** | -0.0254 | +0.4806 | 连续动态正向偏转 |
| **joint_2** | -0.4443 | +0.5979 | 连续动态正向偏转 |
| **joint_3** | -0.6029 | +0.3650 | 连续动态正向偏转 |
| **joint_4** | -0.4084 | -0.0818 | 连续动态回正 |
| **joint_5** | +0.0254 | -0.4806 | 连续动态反向偏转 |
| **joint_6** | +0.4443 | -0.5979 | 连续动态反向偏转 |

六关节均按 `host_teleop` 下发的正弦轨迹及相位差实现平滑闭环跟踪。

### 3.6 退出后进程清理
发送 `SIGINT` 中断后，执行进程检查：
```bash
ps aux | grep -E '(mujoco_bridge_node|host_manager|host_teleop)' | grep -v grep
```
输出：`ALL PROCESSES TERMINATED CLEANLY`，无僵死进程与残留。

---

## 4. 验收标准逐项核对表

| 序号 | 验收标准项 | 实际检查结果 | 结论 |
| :---: | :--- | :--- | :---: |
| 1 | 工作空间构建成功，四个包均可被 ROS 2 发现 | 5.38s 完成构建，4 个包全量发现 | **PASS** |
| 2 | 自定义消息与急停服务接口可正常查询 | `MotorCommand` / `EmergencyStop` 正常显示 | **PASS** |
| 3 | `host_manager`、`mujoco_bridge_node`、`host_teleop` 正常启动 | 3 个节点均在 `ros2 node list` 中存活 | **PASS** |
| 4 | `/host/system_status` 可收到状态消息，运行状态符合预期 | 收到 `state: RUNNING`, 0 错误 | **PASS** |
| 5 | `/bridge/motor_state` 持续收到六关节反馈 | 持续收到，10s 内收到 500 帧 | **PASS** |
| 6 | `/joint_states` 持续收到六关节状态 | 持续收到，10s 内收到 492 帧 | **PASS** |
| 7 | 两个反馈话题实际发布频率接近配置的 50 Hz | 实测 **50.92 Hz** 与 **51.00 Hz** | **PASS** |
| 8 | 六关节状态能对测试轨迹产生响应 | 6 个关节在 2s 前后产生平滑大范围位置变化 | **PASS** |
| 9 | 测试退出后相关 ROS 2 进程正常结束 | 退出后进程列表为空，无残留 | **PASS** |

---

## 5. 最终结论

**结论：PASS**

本轮在 WSL2 / ROS 2 Jazzy 环境下严格执行了只读验收测试，确认当前工作空间已具备完整的 ROS 2 上位机与内部虚拟机械臂闭环通信能力。
