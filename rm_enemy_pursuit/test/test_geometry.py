import math
import pytest
from geometry_msgs.msg import PoseStamped
from rm_enemy_pursuit.geometry import stand_off, valid_pose


@pytest.mark.parametrize('angle', [0, 45, 90, 135, 180, 225, 270, 315])
def test_all_directions_keep_standoff(angle):
    a = math.radians(angle)
    enemy = (3*math.cos(a), 3*math.sin(a))
    x, y, yaw = stand_off((0, 0), enemy, 1)
    assert math.hypot(x-enemy[0], y-enemy[1]) == pytest.approx(1)
    assert math.hypot(x, y) == pytest.approx(2)
    assert math.cos(yaw-a) == pytest.approx(1)


def test_already_close_and_coincident():
    assert stand_off((0, 0), (0.5, 0), 1)[:2] == (0, 0)
    assert stand_off((0, 0), (0, 0), 1) == (0, 0, 0)


def test_invalid_detection():
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.orientation.w = 1.0
    assert valid_pose(p, 'map')
    p.pose.position.x = float('nan')
    assert not valid_pose(p, 'map')
    p.pose.position.x = 0.0
    p.header.frame_id = 'odom'
    assert not valid_pose(p, 'map')
