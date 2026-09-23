#!/usr/bin/env bash
set -eo pipefail
TASK_ROOT="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash
cd "$TASK_ROOT/ros2_ws"
colcon build --symlink-install --packages-select stl_to_grid_map --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select stl_to_grid_map --event-handlers console_direct+
colcon test-result --verbose
