#!/usr/bin/env bash
set -eo pipefail
TASK_ROOT="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$TASK_ROOT/ros2_ws/install/setup.bash"
python3 "$TASK_ROOT/check_map.py"
python3 "$TASK_ROOT/verify_submission.py"
