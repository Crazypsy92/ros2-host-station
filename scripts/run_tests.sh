#!/usr/bin/env bash
# ==============================================================================
# Automated Communication & Host Station Verification Suite
# ==============================================================================

set -e

WS_DIR="/home/bridge/projects/ros2"
cd "$WS_DIR"

source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"

echo "================================================================="
echo "   [TEST 1/2] 测试 MuJoCo 仿真桥接与上位机协同通信闭环...      "
echo "================================================================="

# 后台启动 MuJoCo 闭环
ros2 launch mujoco_bridge test_bridge.launch.py > /tmp/mujoco_test.log 2>&1 &
PID_MUJOCO=$!
sleep 4

echo "--- 检查当前运行中的节点 ---"
ros2 node list

echo "--- 检查当前活跃的话题 ---"
ros2 topic list

echo "--- 获取上位机状态 (/host/system_status) ---"
ros2 topic echo /host/system_status --once

echo "--- 获取仿真器反馈 (/bridge/motor_state) ---"
ros2 topic echo /bridge/motor_state --once

echo "--- 获取标准关节状态 (/joint_states) ---"
ros2 topic echo /joint_states --once

# 停止测试 1
kill $PID_MUJOCO 2>/dev/null || true
pkill -f mujoco_bridge 2>/dev/null || true
pkill -f host_manager 2>/dev/null || true
pkill -f host_teleop 2>/dev/null || true
sleep 2

echo "================================================================="
echo "   [TEST 2/2] 测试 MCU 单片机通信 (UDP/串口回环与下位机固件)     "
echo "================================================================="

# 后台启动 MCU 回环
ros2 launch mcu_bridge test_loopback.launch.py > /tmp/mcu_test.log 2>&1 &
PID_MCU=$!
sleep 4

echo "--- 检查当前运行中的节点 ---"
ros2 node list

echo "--- 获取上位机状态 (/host/system_status) ---"
ros2 topic echo /host/system_status --once

echo "--- 获取单片机反馈状态 (/bridge/motor_state) ---"
ros2 topic echo /bridge/motor_state --once

echo "--- 测试上位机安全急停服务调用 (/host/emergency_stop) ---"
ros2 service call /host/emergency_stop host_interfaces/srv/EmergencyStop "{stop: true, reason: 'Automated E-Stop Verification'}"

sleep 1
echo "--- 检查急停后的上位机状态 ---"
ros2 topic echo /host/system_status --once

# 恢复正常
ros2 service call /host/emergency_stop host_interfaces/srv/EmergencyStop "{stop: false, reason: 'Reset'}"

# 停止测试 2
kill $PID_MCU 2>/dev/null || true
pkill -f mcu_bridge 2>/dev/null || true
pkill -f mcu_virtual_device 2>/dev/null || true
pkill -f host_manager 2>/dev/null || true
pkill -f host_teleop 2>/dev/null || true

echo "================================================================="
echo "   [SUCCESS] 所有上位机与通信测试全部通过！                      "
echo "================================================================="
