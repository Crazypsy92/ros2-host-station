#!/usr/bin/env bash
set -e

WS_DIR="/home/bridge/projects/ros2"
cd "$WS_DIR"

source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"

echo "[1/4] Launching WSL2 Windows TCP Bridge System..."
ros2 launch mujoco_bridge windows_tcp_bridge.launch.py > /tmp/wsl_tcp_launch.log 2>&1 &
LAUNCH_PID=$!
echo "Launch PID: $LAUNCH_PID"

sleep 5

echo "[2/4] Node list check:"
ros2 node list

echo "[3/4] Running 60-second benchmark evaluation..."
python3 "$WS_DIR/scripts/benchmark_60s_tcp.py"

echo "[4/4] Cleaning up ROS 2 processes..."
kill -SIGINT $LAUNCH_PID 2>/dev/null || true
pkill -f windows_tcp_bridge_node 2>/dev/null || true
pkill -f host_manager 2>/dev/null || true
pkill -f host_teleop 2>/dev/null || true
sleep 2

ps aux | grep -E '(windows_tcp_bridge_node|host_manager|host_teleop)' | grep -v grep || echo "ALL PROCESSES TERMINATED CLEANLY"
