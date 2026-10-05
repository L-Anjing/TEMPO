#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

if [[ ! -f "$ROS_SETUP" ]]; then
    echo "missing required file: $ROS_SETUP" >&2
    exit 1
fi

set +u
source "$ROS_SETUP"
set -u
cd "$ROOT_DIR"

# 四个实车包；按包顺序构建，单包并行度可通过环境变量调整。
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
colcon build --base-paths "$ROOT_DIR/src" --packages-up-to navigation \
    --symlink-install --executor sequential --event-handlers console_direct+ \
    "$@" --cmake-args -DCMAKE_BUILD_TYPE=Release
