#!/usr/bin/env bash
set -eo pipefail
TASK_ROOT="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$TASK_ROOT/ros2_ws/install/setup.bash"
mkdir -p "$TASK_ROOT/logs" "$TASK_ROOT/output"
export ROS_LOG_DIR="$TASK_ROOT/logs/ros"
python3 - "$TASK_ROOT" <<'PY'
import os
from pathlib import Path
import subprocess
import sys

root = Path(sys.argv[1])
commands = {
    'map': ['ros2', 'launch', 'stl_to_grid_map', 'convert_rmuc2025.launch.py',
            f'input_file:={root}/data/RMUC2025.STL', f'output_prefix:={root}/output/RMUC2025'],
    'rviz': ['rviz2', '-d', str(root / 'config/slam_hw1.rviz')],
}
for name, command in commands.items():
    pidfile = root / f'logs/{name}.pid'
    if pidfile.exists():
        try:
            pid = int(pidfile.read_text().strip())
            args = Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
            args = [os.fsdecode(arg) for arg in args if arg]
            # Match the actual launch command, not just a possibly reused PID.
            expected = command[1:]
            offset = len(args) - len(expected)
            if (offset >= 1 and args[offset:] == expected
                    and Path(args[offset - 1]).name == command[0]):
                print(f'{name} already running: PID {pid}', flush=True)
                continue
        except (OSError, ValueError):
            pass
        print(f'Ignoring stale {name} PID record', flush=True)
    with (root / f'logs/{name}.log').open('a') as log:
        process = subprocess.Popen(command, cwd=root, stdin=subprocess.DEVNULL,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    pidfile.write_text(str(process.pid))
    print(f'Started {name}: PID {process.pid}', flush=True)
PY
echo "Waiting for /map (up to 60 seconds)..."
python3 "$TASK_ROOT/check_map.py"
