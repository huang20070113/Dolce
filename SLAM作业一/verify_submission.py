"""Reproduce raw map, C++ paths, independent path audit, and model provenance."""
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import struct
import subprocess
import tempfile
import time
from datetime import datetime, timezone

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml

ROOT = Path(__file__).resolve().parent
CONFIG = ROOT / 'ros2_ws/src/stl_to_grid_map/config/rmuc2025.yaml'
BIN = ROOT / 'ros2_ws/install/stl_to_grid_map/lib/stl_to_grid_map'
EVIDENCE = ROOT / 'evidence'
EVIDENCE.mkdir(exist_ok=True)
(ROOT / 'figures').mkdir(exist_ok=True)
config = yaml.safe_load(CONFIG.read_text())['stl_to_grid_map']['ros__parameters']
meta = yaml.safe_load((ROOT / 'output/RMUC2025.yaml').read_text())
assert math.isclose(config['resolution'], meta['resolution'])
resolution = meta['resolution']
final = np.asarray(Image.open(ROOT / 'output/RMUC2025.pgm'))
height, width = final.shape

# Same ROS parameter file as the final map; only inflation is disabled.
# The temporary node uses a separate DDS domain and cannot replace the RViz map.
with tempfile.TemporaryDirectory(prefix='slam_hw1_raw_') as temp:
    prefix = Path(temp) / 'RMUC2025_raw'
    command = [str(BIN / 'stl_to_grid_node'), '--ros-args', '--params-file', str(CONFIG),
               '-p', f'input_file:={ROOT}/data/RMUC2025.STL',
               '-p', f'output_prefix:={prefix}', '-p', 'robot_radius:=0.0']
    env = dict(os.environ, ROS_DOMAIN_ID='91', ROS_LOG_DIR=str(ROOT / 'logs/raw_ros'))
    with (EVIDENCE / 'raw_conversion.log').open('w') as log:
        process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 120
            while 'Generated ' not in (EVIDENCE / 'raw_conversion.log').read_text():
                if process.poll() is not None:
                    raise RuntimeError('Raw converter exited; see raw_conversion.log')
                if time.monotonic() > deadline:
                    raise RuntimeError('Raw conversion timed out')
                time.sleep(0.2)
            for ext in ('.pgm', '.yaml'):
                shutil.copyfile(str(prefix) + ext, ROOT / ('output/RMUC2025_raw' + ext))
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            process.wait(timeout=15)
raw = np.asarray(Image.open(ROOT / 'output/RMUC2025_raw.pgm'))
assert raw.shape == final.shape
free = final > 250
clearance = distance_transform_edt(raw > 250) * resolution

def nearest_free(x, y):
    rows, cols = np.nonzero(free)
    k = np.argmin((rows - y)**2 + (cols - x)**2)
    return int(cols[k]), int(rows[k])

tests = [
    ('cross_field', (round(width * 240 / 340), round(height * 610 / 730)),
     (round(width * 100 / 340), round(height * 120 / 730))),
    ('upper_crossing', nearest_free(width * .2, height * .25), nearest_free(width * .8, height * .25)),
    ('lower_crossing', nearest_free(width * .2, height * .75), nearest_free(width * .8, height * .75)),
]
results = []
fig, axes = plt.subplots(1, 3, figsize=(12, 10), constrained_layout=True)
for ax, (name, start, goal) in zip(axes, tests):
    path_file = ROOT / f'output/path_{name}.csv'
    command = [str(BIN / 'verify_map'), str(ROOT / 'output/RMUC2025.pgm'),
               str(ROOT / 'output/RMUC2025_raw.pgm'), *map(str, (*start, *goal)),
               str(resolution), str(path_file)]
    completed = subprocess.run(command, capture_output=True, text=True, check=True)
    (EVIDENCE / f'{name}.log').write_text(completed.stdout + completed.stderr)
    path = np.loadtxt(path_file, delimiter=',', skiprows=1, dtype=int).reshape(-1, 2)
    assert tuple(path[0]) == start and tuple(path[-1]) == goal
    assert np.all(path[:, 0] >= 0) and np.all(path[:, 0] < width)
    assert np.all(path[:, 1] >= 0) and np.all(path[:, 1] < height)
    assert np.all(free[path[:, 1], path[:, 0]])
    delta = np.diff(path, axis=0)
    assert np.all(np.max(np.abs(delta), axis=1) == 1)
    for (x, y), (dx, dy) in zip(path[:-1], delta):
        if dx and dy:
            assert free[y, x + dx] and free[y + dy, x], 'Diagonal corner cutting'
    length = float(np.linalg.norm(delta, axis=1).sum() * resolution)
    minimum = float(clearance[path[:, 1], path[:, 0]].min())
    assert minimum > config['robot_radius'] - 1e-9
    def world(pixel):
        x, y = pixel
        return [meta['origin'][0] + (x + .5) * resolution,
                meta['origin'][1] + (height - y - .5) * resolution]
    result = {'name': name, 'status': 'PASS', 'start_pgm': start, 'goal_pgm': goal,
              'start_world_m': world(start), 'goal_world_m': world(goal),
              'path_cells': len(path), 'length_m': length,
              'min_obstacle_center_distance_m': minimum,
              'no_corner_cutting': True, 'all_path_cells_free': True}
    results.append(result)
    print(json.dumps(result), flush=True)
    ax.imshow(final, cmap='gray', vmin=0, vmax=255, interpolation='nearest')
    ax.plot(path[:, 0], path[:, 1], color='#e65b20', linewidth=1.6)
    ax.scatter(*start, c='#00a582', s=35, zorder=3, label='Start')
    ax.scatter(*goal, c='#2468de', s=35, zorder=3, label='Goal')
    ax.set_title(f'{name}\n{length:.3f} m | clearance {minimum:.3f} m')
    ax.set_xlabel('PGM column'); ax.set_ylabel('PGM row (top = 0)')
    ax.legend(loc='lower right')
fig.savefig(ROOT / 'figures/path_validation.png', dpi=170)
plt.close(fig)
Image.fromarray(final).save(ROOT / 'figures/final_map.png')
summary = {'checked_at_utc': datetime.now(timezone.utc).isoformat(), 'status': 'PASS',
           'resolution': resolution, 'width': width, 'height': height,
           'robot_radius_m': config['robot_radius'], 'raw_robot_radius_m': 0,
           'free_cells': int(free.sum()), 'occupied_cells': int((~free).sum()),
           'tests': results,
           'scope': 'Discrete geometric path feasibility; not real-robot motion or navigation validation.'}
(EVIDENCE / 'path_validation.json').write_text(json.dumps(summary, indent=2))

model = ROOT / 'data/RMUC2025.STL'
record = json.loads((ROOT / 'data/model_metadata.json').read_text())
digest = hashlib.sha256()
with model.open('rb') as source:
    source.seek(80)
    triangle_count = struct.unpack('<I', source.read(4))[0]
    source.seek(0)
    for block in iter(lambda: source.read(4 * 1024 * 1024), b''):
        digest.update(block)
assert digest.hexdigest() == record['stl_sha256']
assert model.stat().st_size == 84 + 50 * triangle_count == record['stl_bytes']
assert triangle_count == record['triangles']
provenance = {
    'checked_at_utc': datetime.now(timezone.utc).isoformat(),
    'local_stl_integrity': 'PASS', 'source_record': record,
    'actual_stl_sha256': digest.hexdigest(), 'actual_stl_bytes': model.stat().st_size,
    'binary_stl_triangle_count': triangle_count,
    'source_step_available': False, 'assignment_stl_available': False,
    'assignment_stl_match': 'NOT_VERIFIED',
    'explanation': 'Existing metadata records conversion from RMUC2025.stp. Current STL matches that record. The original STEP and the assignment download are unavailable, so equivalence to the assignment file is not claimed.'}
(EVIDENCE / 'model_provenance.json').write_text(json.dumps(provenance, indent=2, ensure_ascii=False))
print('Model hash and binary STL size/count: PASS', flush=True)
