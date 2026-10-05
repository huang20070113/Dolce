"""单一 Nav2 目标所有者：巡航 -> 取消 -> 追踪 -> 取消 -> 恢复巡航。

不向 cmd_vel 发布速度。取消后必须等到旧 action 返回终态再发送新目标，
避免导航目标竞争。普通目标通过 /mission_goal 输入，不直接发送给 Nav2。
"""
import copy
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.clock import Clock, ClockType
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from .geometry import stand_off, valid_pose


class PursuitManager(Node):
    def __init__(self):
        super().__init__('pursuit_manager')
        defaults = dict(global_frame='map', robot_frame='base_link',
                        detection_timeout=2.0, stop_distance=1.0,
                        release_margin=0.3, goal_update_distance=0.5,
                        goal_update_period=1.0, max_pursuit_duration=30.0,
                        tf_timeout=0.8, action_name='navigate_to_pose')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.cfg = {key: self.get_parameter(key).value for key in defaults}
        if any(self.cfg[k] <= 0 for k in defaults if isinstance(defaults[k], float)):
            raise ValueError('All duration and distance parameters must be positive')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.client = ActionClient(self, NavigateToPose, self.cfg['action_name'])
        self.create_subscription(PoseStamped, 'mission_goal', self.mission_cb, 10)
        self.create_subscription(PoseStamped, 'enemy_pose', self.enemy_cb, 10)
        self.status_pub = self.create_publisher(String, 'pursuit_status', 10)
        self.create_service(Trigger, 'clear_mission', self.clear_mission)
        self.mission = None
        self.mission_id = 0
        self.enemy = None
        self.seen = -math.inf
        self.episode = None
        self.suppressed = False
        self.holding = False
        self.active = None
        self.active_key = None
        self.active_pose = None
        self.sending = False
        self.cancel_requested = False
        self.last_send = -math.inf
        self.retry_at = 0.0
        self.state = ''
        self.create_timer(0.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def mission_cb(self, msg):
        if not valid_pose(msg, self.cfg['global_frame']):
            self.get_logger().warning('Rejected mission: need a finite map pose and unit quaternion')
            return
        self.mission = copy.deepcopy(msg)
        self.mission_id += 1

    def clear_mission(self, request, response):
        self.mission = None
        response.success = True
        response.message = 'Saved mission cleared; pursuit, if present, remains active'
        return response

    def enemy_cb(self, msg):
        if not valid_pose(msg, self.cfg['global_frame']):
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = self.now_seconds() - stamp
        if age < -0.2 or age > self.cfg['detection_timeout']:
            return  # 过时/未来消息不触发追踪，时间必须与仿真时钟一致。
        self.enemy = copy.deepcopy(msg)
        self.seen = time.monotonic()

    def status(self, state):
        if state != self.state:
            self.get_logger().info(state)
            self.state = state
        self.status_pub.publish(String(data=state))

    def robot_xy(self):
        try:
            t = self.tf.lookup_transform(self.cfg['global_frame'], self.cfg['robot_frame'], Time())
            stamp = t.header.stamp.sec + t.header.stamp.nanosec / 1e9
            if stamp and abs(self.now_seconds() - stamp) > self.cfg['tf_timeout']:
                return None
            return t.transform.translation.x, t.transform.translation.y
        except TransformException:
            return None

    def desired(self, now):
        # 使用接收端单调时钟监测断流，即使仿真时钟暂停也不会保留过期检测。
        fresh = self.enemy is not None and now - self.seen < self.cfg['detection_timeout']
        if not fresh:
            self.episode, self.suppressed, self.holding = None, False, False
        elif not self.suppressed:
            if self.episode is None:
                self.episode = now
            if now - self.episode >= self.cfg['max_pursuit_duration']:
                self.suppressed = True  # 本轮持续检测到超时；断流后才允许新一轮。
            else:
                robot = self.robot_xy()
                if robot is None:
                    return None, None, 'WAIT_TF'
                enemy = self.enemy.pose.position
                distance = math.hypot(enemy.x - robot[0], enemy.y - robot[1])
                limit = self.cfg['stop_distance'] + (self.cfg['release_margin'] if self.holding else 0)
                self.holding = distance <= limit
                if self.holding:
                    return None, None, 'HOLD'
                x, y, yaw = stand_off(robot, (enemy.x, enemy.y), self.cfg['stop_distance'])
                goal = PoseStamped()
                goal.header.frame_id = self.cfg['global_frame']
                goal.pose.position.x, goal.pose.position.y = x, y
                goal.pose.orientation.z = math.sin(yaw / 2)
                goal.pose.orientation.w = math.cos(yaw / 2)
                return ('enemy',), goal, 'PURSUIT'
        if self.mission:
            return ('mission', self.mission_id), self.mission, 'MISSION'
        return None, None, 'IDLE'

    def tick(self):
        now = time.monotonic()
        key, pose, state = self.desired(now)
        self.status(state)
        if self.sending or self.cancel_requested:
            return
        if self.active:
            replace = key != self.active_key
            if key == ('enemy',) and self.active_key == key:
                a, b = pose.pose.position, self.active_pose.pose.position
                replace = (math.hypot(a.x-b.x, a.y-b.y) >= self.cfg['goal_update_distance']
                           and now-self.last_send >= self.cfg['goal_update_period'])
            if replace:
                self.cancel_requested = True
                self.active.cancel_goal_async().add_done_callback(self.cancel_response)
            return
        if pose is None or now < self.retry_at or not self.client.server_is_ready():
            return
        goal = NavigateToPose.Goal()
        goal.pose = copy.deepcopy(pose)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.active_key, self.active_pose = key, goal.pose
        self.last_send = now
        self.sending = True
        self.client.send_goal_async(goal).add_done_callback(self.accepted)

    def accepted(self, future):
        self.sending = False
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Goal request failed: {exc}')
            self.retry_at = time.monotonic() + 2.0
            return
        if not handle.accepted:
            self.get_logger().warning('Nav2 rejected goal; retry after 2 seconds')
            self.retry_at = time.monotonic() + 2.0
            return
        self.active = handle
        handle.get_result_async().add_done_callback(self.finished)
        # 若等待接收期间检测状态变化，下一轮 tick 将先取消该目标。

    def cancel_response(self, future):
        try:
            if not future.result().goals_canceling:
                self.get_logger().warning('Cancel not accepted; waiting for terminal result')
        except Exception as exc:
            self.get_logger().error(f'Cancel failed: {exc}; waiting for terminal result')
        # 不在这里清空 active：服务端确认取消并不等于旧控制任务已结束。

    def finished(self, future):
        was_cancel = self.cancel_requested
        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().error(f'Action result failed: {exc}')
            status = GoalStatus.STATUS_ABORTED
        if (not was_cancel and status == GoalStatus.STATUS_SUCCEEDED
                and self.active_key == ('mission', self.mission_id)):
            self.mission = None
        if not was_cancel and status == GoalStatus.STATUS_SUCCEEDED and self.active_key == ('enemy',):
            self.holding = True
        self.active = None
        self.cancel_requested = False
        self.retry_at = time.monotonic() + (0.0 if was_cancel else 1.0)


def main(args=None):
    rclpy.init(args=args)
    node = PursuitManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 尽力取消自己的目标；演示结束应同时关闭整个导航 launch。
        if node.active:
            future = node.active.cancel_goal_async()
            rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
