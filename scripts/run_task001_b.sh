#!/usr/bin/env bash
set -e

WS_DIR="/home/bridge/projects/ros2"
cd "$WS_DIR"

source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"

echo "=== 1. LAUNCHING VIRTUAL ARM CLOSED LOOP ==="
ros2 launch mujoco_bridge test_bridge.launch.py > /tmp/task001_b_launch.log 2>&1 &
LAUNCH_PID=$!
echo "Launch PID: $LAUNCH_PID"

# Wait for nodes to initialize
sleep 5

echo ""
echo "=== 2. NODE LIST ==="
ros2 node list

echo ""
echo "=== 3. TOPIC LIST ==="
ros2 topic list

echo ""
echo "=== 4. SYSTEM STATUS SAMPLE ==="
ros2 topic echo /host/system_status --once

echo ""
echo "=== 5. JOINT STATES SAMPLE @ T0 ==="
ros2 topic echo /joint_states --once

sleep 2

echo ""
echo "=== 6. JOINT STATES SAMPLE @ T0+2s ==="
ros2 topic echo /joint_states --once

echo ""
echo "=== 7. MEASURING /bridge/motor_state HZ (10s window) ==="
python3 -c "
import rclpy, time
from host_interfaces.msg import MotorState
rclpy.init()
node = rclpy.create_node('hz_eval_motor')
timestamps = []
def cb(msg):
    timestamps.append(time.time())
sub = node.create_subscription(MotorState, '/bridge/motor_state', cb, 100)
t_start = time.time()
while time.time() - t_start < 10.0:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()
if len(timestamps) > 1:
    dt = timestamps[-1] - timestamps[0]
    rate = (len(timestamps) - 1) / dt
    print(f'Average rate: {rate:.2f} Hz, received {len(timestamps)} msgs over {dt:.2f} s')
else:
    print('No messages received!')
"

echo ""
echo "=== 8. MEASURING /joint_states HZ (10s window) ==="
python3 -c "
import rclpy, time
from sensor_msgs.msg import JointState
rclpy.init()
node = rclpy.create_node('hz_eval_joint')
timestamps = []
def cb(msg):
    timestamps.append(time.time())
sub = node.create_subscription(JointState, '/joint_states', cb, 100)
t_start = time.time()
while time.time() - t_start < 10.0:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()
if len(timestamps) > 1:
    dt = timestamps[-1] - timestamps[0]
    rate = (len(timestamps) - 1) / dt
    print(f'Average rate: {rate:.2f} Hz, received {len(timestamps)} msgs over {dt:.2f} s')
else:
    print('No messages received!')
"

echo ""
echo "=== 9. TERMINATING PROCESSES ==="
kill -SIGINT $LAUNCH_PID 2>/dev/null || true
pkill -f mujoco_bridge_node 2>/dev/null || true
pkill -f host_manager 2>/dev/null || true
pkill -f host_teleop 2>/dev/null || true
sleep 2

echo ""
echo "=== 10. POST-TERMINATION PROCESS CHECK ==="
ps aux | grep -E '(mujoco_bridge_node|host_manager|host_teleop)' | grep -v grep || echo "ALL PROCESSES TERMINATED CLEANLY"
