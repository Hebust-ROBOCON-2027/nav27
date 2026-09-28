#!/usr/bin/env bash
# ==============================================================================
# 2027 赛场仿真一键启动脚本 (Ignition Gazebo + AT_R2)
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"

DO_BUILD=0
CAMP="red"
EXTRA_ARGS=()

usage() {
  echo "用法: $(basename "$0") [选项] [-- <透传给 ros2 launch 的参数>]"
  echo ""
  echo "选项:"
  echo "  -b, --build       启动前先 colcon build 仿真工作空间"
  echo "  --red             红方出发点 (默认, (-4.75, -5.15))"
  echo "  --blue            蓝方出发点 (4.75, -5.15)"
  echo "  -h, --help        显示帮助信息"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -b|--build)
      DO_BUILD=1
      shift
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
  echo "[INFO] 正在编译仿真工作空间..."
  cd "${SCRIPT_DIR}"
  colcon build --symlink-install --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_BUILD_TYPE=Release
fi

source "${SCRIPT_DIR}/install/setup.bash"

# 清理可能残留的仿真进程，避免端口冲突
pkill -9 -f "ign gazebo" 2>/dev/null || true
pkill -9 -f "parameter_bridge" 2>/dev/null || true
sleep 0.5

# 根据阵营选择 world 配置
WORLD="rc_2027"
if [ "$CAMP" = "blue" ]; then
  WORLD="rc_2027_blue"
elif [ "$CAMP" = "red" ]; then
  WORLD="rc_2027_red"
fi

# 更新 gz_world.yaml 中的当前选择
GZ_WORLD_YAML="${SCRIPT_DIR}/src/rmu_gazebo_simulator/rmu_gazebo_simulator/config/gz_world.yaml"
if [ -f "$GZ_WORLD_YAML" ]; then
  sed -i "s/^world:.*/world: "${WORLD}"/" "$GZ_WORLD_YAML"
fi

echo "================================================================="
echo "  启动 2027 赛场仿真 (Ignition Gazebo) — 阵营: ${CAMP} (${WORLD})"
echo "================================================================="

exec ros2 launch rmu_gazebo_simulator bringup_sim.launch.py "${EXTRA_ARGS[@]}"
