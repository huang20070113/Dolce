"""真实 DDS + rclpy action 通信；服务端模拟 Nav2，不代表 Gazebo 行驶测试。"""
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import TransformBroadcaster
from rm_enemy_pursuit.manager import PursuitManager


def test_action_arbitration():
    rclpy.init()
    manager = PursuitManager()
    manager.cfg.update(detection_timeout=0.5, goal_update_period=0.1,
                       tf_timeout=2.0, max_pursuit_duration=20.0)
    driver = Node('pursuit_test_driver')
    pub = driver.create_publisher(PoseStamped, 'enemy_pose', 10)
    mission = driver.create_publisher(PoseStamped, 'mission_goal', 10)
    tf = TransformBroadcaster(driver)
    events = []
    running = [0]
    max_running = [0]
    abort = [False]
    reject = [False]

    def execute(handle):
        running[0] += 1
        max_running[0] = max(max_running[0], running[0])
        p = handle.request.pose.pose.position
        events.append(('start', p.x, p.y))
        while rclpy.ok() and not handle.is_cancel_requested and not abort[0]:
            time.sleep(0.02)
        if handle.is_cancel_requested:
            time.sleep(0.15)  # 放大取消服务答复到终态之间的间隔，检查是否出现目标重叠。
            handle.canceled()
        else:
            handle.abort()
            abort[0] = False
        running[0] -= 1
        events.append(('end', p.x, p.y))
        return NavigateToPose.Result()

    def goal_cb(request):
        if reject[0]:
            reject[0] = False
            events.append(('reject', 0, 0))
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    server = ActionServer(driver, NavigateToPose, 'navigate_to_pose', execute,
                          goal_callback=goal_cb,
                          cancel_callback=lambda _: CancelResponse.ACCEPT,
                          callback_group=ReentrantCallbackGroup())
    def broadcast():
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = 'map', 'base_link'
        t.header.stamp = driver.get_clock().now().to_msg()
        t.transform.rotation.w = 1.0
        tf.sendTransform(t)
    timer = driver.create_timer(0.05, broadcast)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(manager)
    executor.add_node(driver)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def pose(x, y, frame='map', old=False):
        msg = PoseStamped()
        msg.header.frame_id = frame
        msg.header.stamp = driver.get_clock().now().to_msg()
        if old:
            msg.header.stamp.sec -= 20
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        msg.pose.orientation.w = 1.0
        return msg

    def wait(predicate, timeout=5, publish=None):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            if publish:
                publish()
            if predicate():
                return
            time.sleep(0.04)
        raise AssertionError(f'Timeout; state={manager.state}; events={events}')

    def active_at(x, y):
        return (manager.active is not None and not manager.cancel_requested
                and abs(manager.active_pose.pose.position.x-x)<0.05
                and abs(manager.active_pose.pose.position.y-y)<0.05)

    try:
        wait(lambda: mission.get_subscription_count()>0 and manager.client.server_is_ready())
        mission.publish(pose(8, 0))
        wait(lambda: active_at(8, 0))
        # 无效坐标系和旧检测不抢占普通任务。
        pub.publish(pose(3, 0, 'odom'))
        pub.publish(pose(3, 0, old=True))
        time.sleep(0.3)
        assert active_at(8, 0)
        # 四个方向均应触发追踪，并与模拟 Nav2 完成串行取消。
        for x, y in [(3, 0), (0, 3), (-3, 0), (0, -3)]:
            wait(lambda: active_at(x*2/3, y*2/3), publish=lambda: pub.publish(pose(x,y)))
        # 追踪期间新普通目标只更新待恢复任务，不抢占追踪。
        mission.publish(pose(9, 1))
        wait(lambda: manager.mission_id == 2, publish=lambda: pub.publish(pose(0,-3)))
        assert manager.active_key == ('enemy',)
        # 目标进入距离阈值后停止，消息消失后恢复新的普通目标。
        wait(lambda: manager.state == 'HOLD' and manager.active is None,
             publish=lambda: pub.publish(pose(0.4, 0)))
        wait(lambda: active_at(9, 1))
        # 导航失败可重试；目标拒绝也会退避重试。
        reject[0] = True
        abort[0] = True
        wait(lambda: any(e[0]=='reject' for e in events))
        wait(lambda: active_at(9, 1), timeout=5)
        # 丢失 TF 时取消追踪，不凭旧位姿继续发送目标。
        original_robot_xy = manager.robot_xy
        manager.robot_xy = lambda: None
        wait(lambda: manager.state=='WAIT_TF' and manager.active is None,
             publish=lambda: pub.publish(pose(3,0)))
        manager.robot_xy = original_robot_xy
        wait(lambda: active_at(2,0), publish=lambda: pub.publish(pose(3,0)))
        # 持续检测达到时限后恢复普通导航，不反复抢占。
        manager.cfg['max_pursuit_duration'] = 0.8
        wait(lambda: active_at(9,1), publish=lambda: pub.publish(pose(3,0)))
        assert manager.suppressed
        assert max_running[0] == 1
        print('PASS: cardinal directions, stale/invalid detection, hold, resume, new mission, abort/reject retry, TF loss, pursuit timeout; max concurrent goals=1')
    finally:
        if manager.active:
            manager.active.cancel_goal_async()
        time.sleep(0.3)
        abort[0] = True
        executor.shutdown(timeout_sec=3)
        server.destroy()
        manager.destroy_node()
        driver.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2)
