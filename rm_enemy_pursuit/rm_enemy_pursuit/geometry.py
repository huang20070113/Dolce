"""与 ROS 无关的几何运算，单位为米和弧度。"""
import math


def stand_off(robot, enemy, distance):
    """沿连线留出距离；近于阈值时不倒车追逐。返回 x、y、朝向。"""
    dx, dy = enemy[0] - robot[0], enemy[1] - robot[1]
    length = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    travel = max(0.0, length - distance)
    if length < 1e-9:
        return robot[0], robot[1], 0.0
    return robot[0] + dx / length * travel, robot[1] + dy / length * travel, yaw


def valid_pose(msg, frame):
    p, q = msg.pose.position, msg.pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    return (msg.header.frame_id == frame and all(math.isfinite(v) for v in values)
            and abs(sum(v*v for v in (q.x, q.y, q.z, q.w)) - 1.0) < 0.05)
