"""Check the live ROS map against configuration and saved PGM/YAML."""
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from PIL import Image
import yaml
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


def check(root):
    config = yaml.safe_load((root / 'ros2_ws/src/stl_to_grid_map/config/rmuc2025.yaml').read_text())['stl_to_grid_map']['ros__parameters']
    rclpy.init()
    node = rclpy.create_node('slam_hw1_runtime_check')
    messages = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sub = node.create_subscription(OccupancyGrid, '/map', messages.append, qos)
    try:
        deadline = time.monotonic() + 60
        while len(messages) < 2 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=1)
        if len(messages) < 2:
            raise RuntimeError('Did not receive two /map messages within 60 seconds')
        message = messages[-1]
        # Conversion writes the files before it starts publishing.
        meta = yaml.safe_load((root / 'output/RMUC2025.yaml').read_text())
        pixels = np.asarray(Image.open(root / 'output' / meta['image']).convert('L'))
        height, width = pixels.shape
        expected = np.where(np.flipud(pixels) < 128, 100, 0).ravel()
        if not math.isclose(meta['resolution'], config['resolution'], abs_tol=1e-8):
            raise RuntimeError('Saved map resolution differs from configuration; restart conversion')
        def require(condition, explanation):
            if not condition:
                raise RuntimeError(explanation)
        require(message.header.frame_id == config['frame_id'], 'Frame mismatch')
        require((message.info.width, message.info.height) == (width, height), 'PGM/live dimensions differ')
        require(math.isclose(message.info.resolution, meta['resolution'], abs_tol=1e-7), 'Live resolution mismatch')
        require(len(message.data) == width * height, 'Invalid data length')
        require(np.array_equal(message.data, expected), 'Live /map cells differ from saved PGM')
        require(0 in message.data and 100 in message.data, 'Map needs both free and occupied cells')
        require(math.isclose(message.info.origin.position.x, meta['origin'][0], abs_tol=1e-7), 'Origin X mismatch')
        require(math.isclose(message.info.origin.position.y, meta['origin'][1], abs_tol=1e-7), 'Origin Y mismatch')
        publishers = node.get_publishers_info_by_topic('/map')
        require(len(publishers) == 1, 'Expected exactly one map publisher')
        stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in messages]
        require(stamps[-1] > stamps[0], 'Map timestamps are not advancing')
        result = {
            'checked_at_utc': datetime.now(timezone.utc).isoformat(), 'status': 'PASS',
            'frame': message.header.frame_id, 'width': width, 'height': height,
            'resolution': message.info.resolution, 'origin': meta['origin'],
            'received_messages': len(messages), 'message_interval_seconds': stamps[-1] - stamps[-2],
            'publishers': [i.node_name for i in publishers],
            'subscribers': [i.node_name for i in node.get_subscriptions_info_by_topic('/map')],
            'occupied_cells': sum(v == 100 for v in message.data),
            'free_cells': sum(v == 0 for v in message.data),
            'live_cells_equal_pgm': True,
        }
        (root / 'logs').mkdir(exist_ok=True)
        (root / 'logs/runtime_check.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    check(Path(__file__).resolve().parent)
