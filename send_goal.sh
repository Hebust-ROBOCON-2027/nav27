#!/usr/bin/env bash
# ==============================================================================
# 交互式导航目标点发送器 (回车立即开启自动导航)
# 用法: ./send_goal.sh [--ns <namespace>]
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

python3 "${SCRIPT_DIR}/tools/send_nav_goal.py" "$@"
