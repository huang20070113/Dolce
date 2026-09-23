#!/usr/bin/env bash
set -eo pipefail
TASK_ROOT="$(cd "$(dirname "$0")" && pwd)"
python3 - "$TASK_ROOT" "${1:-all}" <<'PY'
import os
from pathlib import Path
import signal
import sys
import time
root = Path(sys.argv[1])
choice = sys.argv[2]
if choice not in ('all', 'map', 'rviz'):
    raise SystemExit('Usage: bash stop.sh [all|map|rviz]')
expected = {
    'map': ['ros2', 'launch', 'stl_to_grid_map', 'convert_rmuc2025.launch.py',
            f'input_file:={root}/data/RMUC2025.STL', f'output_prefix:={root}/output/RMUC2025'],
    'rviz': ['rviz2', '-d', str(root / 'config/slam_hw1.rviz')],
}
for name, command in expected.items():
    if choice != 'all' and name != choice:
        continue
    try:
        pid = int((root / f'logs/{name}.pid').read_text())
        args = [os.fsdecode(a) for a in Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0') if a]
        offset = len(args) - len(command[1:])
        if not (offset >= 1 and args[offset:] == command[1:] and Path(args[offset - 1]).name == command[0]):
            print(f'{name}: stale PID record; no process signalled')
            continue
        # start.sh creates a new session; do not signal any unrelated process group.
        if os.getpgid(pid) != pid:
            raise RuntimeError(f'{name}: unexpected process group; stop it in its launch terminal')
        os.killpg(pid, signal.SIGINT)
        deadline = time.monotonic() + 15
        while Path(f'/proc/{pid}/cmdline').exists():
            if not Path(f'/proc/{pid}/cmdline').read_bytes():
                break
            if time.monotonic() > deadline:
                raise RuntimeError(f'{name} did not stop within 15 seconds')
            time.sleep(.1)
        print(f'{name}: stopped')
    except (FileNotFoundError, ProcessLookupError):
        print(f'{name}: not running')
PY
