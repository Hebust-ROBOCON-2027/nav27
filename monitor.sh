#!/usr/bin/env bash
# ==============================================================================
# 机器人实时状态监控 (map 坐标系位姿 + X/Y 速度)
# 用法: ./monitor.sh [--ns <namespace>]
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

if [ -f "${SCRIPT_DIR}/nav/install/setup.bash" ]; then
  source "${SCRIPT_DIR}/nav/install/setup.bash"
fi

python3 "${SCRIPT_DIR}/tools/monitor_robot_state.py" "$@"
