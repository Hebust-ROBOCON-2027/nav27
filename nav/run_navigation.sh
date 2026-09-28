#!/usr/bin/env bash
# ==============================================================================
# 2027 赛场导航系统一键启动脚本 (Nav2 + MPPI + Point-LIO + GICP + RViz2)
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"

DO_BUILD=0
PACKAGES=""
CAMP="red"
EXTRA_ARGS=()

usage() {
  echo "用法: $(basename "$0") [选项] [-- <透传给 ros2 launch 的参数>]"
  echo ""
  echo "选项:"
  echo "  -b, --build       启动前先 colcon build 导航工作空间"
  echo "  -p, --packages    仅 build 指定的包，例如: -p \"at_r2_nav_bringup point_lio\""
  echo "  --red             红方出发点配置 (默认, (-4.75, -5.15))"
  echo "  --blue            蓝方出发点配置 (4.75, -5.15)"
  echo "  -h, --help        显示帮助信息"
  echo "  --                后续参数原样传给 ros2 launch"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -b|--build)
      DO_BUILD=1
      shift
      ;;
    -p|--packages)
      PACKAGES="$2"
      DO_BUILD=1
      shift 2
      ;;
    --red)
      CAMP="red"
      shift
      ;;
    --blue)
      CAMP="blue"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      EXTRA_ARGS=("$@")
      break
      ;;
    *)
      echo "未知参数: $1"
      usage
      exit 1
      ;;
  esac
done

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

if [ $DO_BUILD -eq 1 ] || [ ! -d "${SCRIPT_DIR}/install" ]; then
  cd "${SCRIPT_DIR}"
  if [ -n "${PACKAGES}" ]; then
    echo "[INFO] 正在编译指定功能包: ${PACKAGES} ..."
    colcon build --symlink-install --packages-select ${PACKAGES} --cmake-args -DCMAKE_BUILD_TYPE=Release
  else
    echo "[INFO] 正在全量编译导航工作空间..."
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
  fi
fi

source "${SCRIPT_DIR}/install/setup.bash"

echo "================================================================="
echo "  启动 2027 赛场仿真导航 (Nav2 MPPI) — 阵营: ${CAMP}"
echo "================================================================="

exec ros2 launch at_r2_nav_bringup at_navigation_simulation_launch.py camp:="${CAMP}" use_sim_time:=true "${EXTRA_ARGS[@]}"
