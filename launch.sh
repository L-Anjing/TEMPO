#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
NAV_SETUP="$ROOT_DIR/install/setup.bash"
NAV_PARAMS="$ROOT_DIR/src/navigation/params/online_planner.yaml"
POINTLIO_PARAMS="$ROOT_DIR/src/point_lio/config/mapping/mid360.yaml"
DRIVER_PARAMS="$ROOT_DIR/src/livox_ros_driver2/config/MID360_params.yaml"
LIDAR_CONFIG="$ROOT_DIR/src/livox_ros_driver2/config/MID360_config.json"
RVIZ_CONFIG="$ROOT_DIR/src/navigation/config/online_navigation.rviz"

# 回放 rosbag 时设 USE_SIM_TIME=1，并用 ros2 bag play <bag> --clock。
if [ "${USE_SIM_TIME:-0}" -eq 1 ]; then SIM_TIME=true; else SIM_TIME=false; fi

# 默认只在本机通信；跨机 MPC 可用 ROS_LOCALHOST_ONLY=0 ./launch.sh。
ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

# Batch-LIO 的 batch 内 OpenMP 并行：OpenMP 默认 ACTIVE 自旋会与主线程抢核，
# 在受限核数下反而把吞吐拉低一个数量级；PASSIVE 让工作线程在并行区之间休眠。
export OMP_WAIT_POLICY="${OMP_WAIT_POLICY:-PASSIVE}"

for required in "$ROS_SETUP" "$NAV_SETUP" "$NAV_PARAMS" "$POINTLIO_PARAMS" "$DRIVER_PARAMS" "$LIDAR_CONFIG" "$RVIZ_CONFIG"; do
  if [[ ! -f "$required" ]]; then
    echo "missing required file: $required" >&2
    exit 1
  fi
done

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "gnome-terminal is not installed" >&2
  exit 1
fi

# 每个节点开一个独立窗口；命令结束后保留日志，按回车关闭。
launch_in_terminal() {
  local title="$1"
  local cmd="$2"
  echo "[$title] launching..."
  if ! gnome-terminal --title="$title" -- bash -c '
    source "$1"
    source "$2"
    cd "$3" || exit 1
    export RC_NAV_ROOT="$3"
    export SIM_TIME="$4"
    export ROS_LOCALHOST_ONLY="$5"
    title="$6"
    echo "===== $title ====="
    bash -c "$7"
    echo
    echo "[$title] 进程已退出，按回车键关闭窗口..."
    read -r
  ' rc_nav "$ROS_SETUP" "$NAV_SETUP" "$ROOT_DIR" "$SIM_TIME" "$ROS_LOCALHOST_ONLY" "$title" "$cmd"; then
    echo "failed to open terminal window: $title" >&2
  fi
  sleep 1
}

echo "Starting step-by-step chain in gnome-terminal windows..."

# launch_in_terminal "01_mid360_driver" \
#   'ros2 launch livox_ros_driver2 msg_MID360_launch.py use_sim_time:="$SIM_TIME" driver_params_file:="$RC_NAV_ROOT/src/livox_ros_driver2/config/MID360_params.yaml" lidar_config_file:="$RC_NAV_ROOT/src/livox_ros_driver2/config/MID360_config.json"'

launch_in_terminal "02_point_lio" \
  'ros2 launch point_lio mapping_mid360.launch.py use_sim_time:="$SIM_TIME" params_file:="$RC_NAV_ROOT/src/point_lio/config/mapping/mid360.yaml" rviz:=false'

launch_in_terminal "03_navigation" \
  'ros2 launch navigation online_navigation.launch.py use_sim_time:="$SIM_TIME" params_file:="$RC_NAV_ROOT/src/navigation/params/online_planner.yaml" pointlio_params_file:="$RC_NAV_ROOT/src/point_lio/config/mapping/mid360.yaml" rviz:=true rviz_config_file:="$RC_NAV_ROOT/src/navigation/config/online_navigation.rviz"'

echo "All requested terminals have been opened."
