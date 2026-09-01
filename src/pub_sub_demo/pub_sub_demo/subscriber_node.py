#!/usr/bin/env python3
"""
ROS2 订阅者节点演示
功能：订阅并处理消息
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class SubscriberNode(Node):
    """订阅者类，继承自Node"""

    def __init__(self):
        """初始化节点"""
        super().__init__('listener_node')

        # 创建订阅者
        # 参数1：消息类型 (std_msgs.msg.Int32)
        # 参数2：话题名称 (/counter)
        # 参数3：回调函数 (self.listener_callback)
        # 参数4：队列大小 (10)
        self.subscription = self.create_subscription(
            Int32,
            'counter',
            self.listener_callback,
            10
        )

        # 初始化消息计数
        self.message_count = 0

        # 打印初始化完成信息
        self.get_logger().info('订阅者节点已启动，等待消息...')

    def listener_callback(self, msg):
        """
        订阅回调函数
        当收到话题消息时自动执行

        参数：
            msg: 接收到的消息对象 (std_msgs.msg.Int32)
        """
        # 消息计数
        self.message_count += 1

        # 提取消息数据
        counter_value = msg.data

        # 打印接收到的消息
        self.get_logger().info(
            f'收到消息 #{self.message_count}: counter = {counter_value}'
        )

        # 可以在这里进行任意处理逻辑
        if counter_value % 5 == 0:
            self.get_logger().warn(f'检测到5的倍数: {counter_value}')

def main(args=None):
    """节点主函数"""
    # 初始化ROS2
    rclpy.init(args=args)

    # 创建节点实例
    subscriber_node = SubscriberNode()

    try:
        # 循环运行节点，处理回调函数
        rclpy.spin(subscriber_node)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        subscriber_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
