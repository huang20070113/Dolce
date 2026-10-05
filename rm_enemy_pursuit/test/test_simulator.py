"""通过真实 TF 和话题验证模拟目标出现在后方，并固定于 map 坐标系。"""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from rm_enemy_pursuit.simulator import EnemySimulator


def test_simulated_detection_is_anchored_in_map():
    rclpy.init()
    sim = EnemySimulator()
    sim.set_parameters([Parameter('start_delay',value=0.0),
                        Parameter('bearing_deg',value=180.0),
                        Parameter('visible_seconds',value=20.0)])
    node = Node('sim_detector_test')
    broadcaster = TransformBroadcaster(node)
    seen=[]
    sub=node.create_subscription(PoseStamped,'enemy_pose',seen.append,10)
    try:
        end=time.monotonic()+4
        while time.monotonic()<end and len(seen)<3:
            t=TransformStamped();t.header.frame_id='map';t.child_frame_id='base_link'
            t.header.stamp=node.get_clock().now().to_msg();t.transform.rotation.w=1.0
            broadcaster.sendTransform(t)
            rclpy.spin_once(sim,timeout_sec=.02);rclpy.spin_once(node,timeout_sec=.02)
        assert len(seen)>=3
        first=seen[-1]
        assert abs(first.pose.position.x+3)<1e-6
        assert abs(first.pose.position.y)<1e-6
        for i in range(15):
            t.transform.translation.x=2.0;t.header.stamp=node.get_clock().now().to_msg()
            broadcaster.sendTransform(t)
            rclpy.spin_once(sim,timeout_sec=.02);rclpy.spin_once(node,timeout_sec=.02)
        assert abs(seen[-1].pose.position.x+3)<1e-6
        assert seen[-1].pose.position.z==0.0  # Marker 的显示高度不能改写检测消息。
    finally:
        sim.destroy_node();node.destroy_node();rclpy.shutdown()
