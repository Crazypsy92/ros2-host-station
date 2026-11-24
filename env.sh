#!/usr/bin/env bash
# ==============================================================================
# ROS 2 Jazzy Environment & Workspace Auto-Activator
# Usage: source env.sh
# ==============================================================================

WS_DIR="/home/bridge/projects/ros2"

# 1. Source ROS 2 base installation
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    export ROS_DISTRO=jazzy
else
    echo "[WARN] /opt/ros/jazzy/setup.bash not found!"
fi

# 2. Source current workspace overlay if built
if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
    echo "[INFO] Sourced workspace overlay: ${WS_DIR}/install/setup.bash"
fi

# 3. Useful shortcuts and aliases
alias ws="cd ${WS_DIR}"
alias cb="colcon build --symlink-install"
alias cbs="colcon build --symlink-install && source ${WS_DIR}/install/setup.bash"
alias cb-pkg="colcon build --symlink-install --packages-select"
alias clean-ws="rm -rf ${WS_DIR}/build ${WS_DIR}/install ${WS_DIR}/log"
alias ros-nodes="ros2 node list"
alias ros-topics="ros2 topic list"

echo "=================================================="
echo "  ROS 2 Host Station Workspace Loaded!            "
echo "  Distro : $ROS_DISTRO                            "
echo "  Path   : $WS_DIR                                "
echo "  Aliases: cb, cbs, clean-ws, ws, ros-nodes       "
echo "=================================================="
