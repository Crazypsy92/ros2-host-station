#!/usr/bin/env bash
# ==============================================================================
# ROS 2 Host Controller Management Script (上位机综合管理脚本)
# ==============================================================================

set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR"

# Source environment
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi
if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
fi

print_help() {
    echo "=================================================================="
    echo "           ROS 2 上位机与通信测试管理工具 (Manager CLI)           "
    echo "=================================================================="
    echo "使用方法: ./manage.sh [命令] [参数]"
    echo ""
    echo "核心构建与环境:"
    echo "  build                 - 编译工作空间 (colcon build --symlink-install)"
    echo "  clean                 - 清理构建产物 (build / install / log)"
    echo "  rebuild               - 完全清理并重新编译"
    echo ""
    echo "上位机与通信测试:"
    echo "  test-mujoco           - 启动 MuJoCo 仿真桥接与上位机协同测试"
    echo "  test-mcu-loopback     - 启动 MCU 虚拟串口回环与下位机通信测试"
    echo "  launch-host           - 启动上位机核心监控与状态机系统"
    echo "  launch-all            - 启动完整上位机（核心 + 仿真 + 硬件桥接）"
    echo ""
    echo "调试与监测:"
    echo "  monitor               - 启动实时上位机终端看板 (CLI Dashboard)"
    echo "  teleop                - 启动简易键盘/指令控制台"
    echo "  nodes                 - 列出当前运行中的 ROS 2 节点"
    echo "  topics                - 列出当前活跃的 ROS 2 话题"
    echo "  info                  - 显示系统环境与状态概览"
    echo "=================================================================="
}

cmd="${1:-help}"

case "$cmd" in
    build)
        echo "[INFO] 开始编译 ROS 2 工作空间..."
        colcon build --symlink-install
        echo "[OK] 编译完成！运行 'source env.sh' 载入环境变量。"
        ;;
    clean)
        echo "[WARN] 正在清理 build / install / log ..."
        rm -rf "$WS_DIR/build" "$WS_DIR/install" "$WS_DIR/log"
        echo "[OK] 清理完成。"
        ;;
    rebuild)
        "$0" clean
        "$0" build
        ;;
    test-mujoco)
        echo "[INFO] 启动 MuJoCo 桥接与上位机节点测试..."
        ros2 launch mujoco_bridge test_bridge.launch.py
        ;;
    test-windows-tcp)
        echo "[INFO] 启动 Windows ↔ WSL2 TCP 跨系统通信测试..."
        ros2 launch mujoco_bridge windows_tcp_bridge.launch.py
        ;;
    test-mcu-loopback)
        echo "[INFO] 启动 MCU 串口/UDP 回环通信测试..."
        ros2 launch mcu_bridge test_loopback.launch.py
        ;;
    launch-host)
        echo "[INFO] 启动上位机核心控制与监控系统..."
        ros2 launch host_controller host_system.launch.py
        ;;
    launch-all)
        echo "[INFO] 启动完整上位机系统 (Host + MuJoCo Bridge + MCU Bridge)..."
        ros2 launch host_controller full_station.launch.py
        ;;
    monitor)
        echo "[INFO] 启动上位机实时遥测监控终端..."
        ros2 run host_controller host_monitor
        ;;
    teleop)
        echo "[INFO] 启动上位机交互控制终端..."
        ros2 run host_controller host_teleop
        ;;
    nodes)
        ros2 node list
        ;;
    topics)
        ros2 topic list
        ;;
    info)
        echo "工作空间目录: $WS_DIR"
        echo "ROS 2 版本: ${ROS_DISTRO:-unknown}"
        echo "已编译包列表:"
        ros2 pkg list | grep -E 'host_|mujoco_|mcu_' || echo "尚未编译上位机包，请先运行 ./manage.sh build"
        ;;
    help|--help|-h)
        print_help
        ;;
    *)
        echo "[ERROR] 未知命令: $cmd"
        print_help
        exit 1
        ;;
esac
