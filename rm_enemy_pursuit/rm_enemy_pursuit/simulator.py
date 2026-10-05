"""模拟检测，不解析相机图像。每轮在机器人任意方位生成一个固定 map 目标。"""
import copy
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener, TransformException


class EnemySimulator(Node):
    def __init__(self):
        super().__init__('enemy_simulator')
        for key, value in dict(global_frame='map', robot_frame='base_link',
                               bearing_deg=180.0, distance=3.0,
                               visible_seconds=8.0, hidden_seconds=6.0,
                               start_delay=10.0, enabled=True).items():
            self.declare_parameter(key, value)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(PoseStamped, 'enemy_pose', 10)
        self.markers = self.create_publisher(Marker, 'enemy_marker', 10)
        self.create_service(SetBool, 'enemy_simulator/enable', self.enable)
        self.enabled = self.get_parameter('enabled').value
        self.origin = time.monotonic()
        self.cycle = -1
        self.pose = None
        self.create_timer(0.1, self.tick)

    def value(self, name):
        return self.get_parameter(name).value

    def enable(self, request, response):
        self.enabled = request.data
        self.origin, self.cycle, self.pose = time.monotonic(), -1, None
        response.success = True
        response.message = 'Detection enabled' if self.enabled else 'Detection disabled'
        return response

    def tick(self):
        visible, hidden = self.value('visible_seconds'), self.value('hidden_seconds')
        elapsed = time.monotonic() - self.origin - self.value('start_delay')
        if not self.enabled or elapsed < 0 or visible <= 0 or hidden < 0:
            return
        cycle = int(elapsed / (visible + hidden))
        if elapsed % (visible + hidden) >= visible:
            return
        if cycle != self.cycle or self.pose is None:
            try:
                t = self.tf.lookup_transform(self.value('global_frame'), self.value('robot_frame'), Time())
            except TransformException:
                return
            stamp = t.header.stamp.sec + t.header.stamp.nanosec / 1e9
            if stamp and abs(self.get_clock().now().nanoseconds / 1e9 - stamp) > 0.8:
                return
            q = t.transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            angle = yaw + math.radians(self.value('bearing_deg'))
            self.pose = PoseStamped()
            self.pose.header.frame_id = self.value('global_frame')
            self.pose.pose.position.x = t.transform.translation.x + self.value('distance')*math.cos(angle)
            self.pose.pose.position.y = t.transform.translation.y + self.value('distance')*math.sin(angle)
            self.pose.pose.orientation.w = 1.0
            self.cycle = cycle  # 固定在 map 中，不随机器人一起移动。
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.pose)
        marker = Marker()
        marker.header = self.pose.header
        marker.ns, marker.id = 'simulated_enemy', 0
        marker.type, marker.action = Marker.SPHERE, Marker.ADD
        marker.pose = copy.deepcopy(self.pose.pose)
        marker.pose.position.z = 0.4
        marker.scale.x = marker.scale.y = marker.scale.z = 0.4
        marker.color.r, marker.color.a = 1.0, 0.9
        marker.lifetime.sec = 1
        self.markers.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = EnemySimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
