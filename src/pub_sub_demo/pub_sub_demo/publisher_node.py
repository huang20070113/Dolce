#!/usr/bin/env python3
"""
ROS2 发布者节点演示
功能：定时发送整数消息
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time

class PublisherNode(Node):
    """发布者类，继承自Node"""

    def __init__(self):
        """初始化节点"""
        super().__init__('talker_node')  # 节点名称

        # 创建发布者
        # 参数1：消息类型 (std_msgs.msg.Int32)
        # 参数2：话题名称 (/counter)
        # 参数3：队列大小 (10)
        self.publisher_ = self.create_publisher(
            Int32,
            'counter',
            10
        )

        # 创建定时器，每秒执行一次回调函数
        # 参数1：时间间隔（秒）
        # 参数2：回调函数
        timer_period = 1.0  # 1秒
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # 计数器
        self.counter = 0

        # 打印初始化完成信息
        self.get_logger().info('发布者节点已启动')

    def timer_callback(self):
        """定时回调函数，定时发送消息"""
        # 创建消息对象
        msg = Int32()
        msg.data = self.counter

        # 发布消息
        self.publisher_.publish(msg)

        # 打印日志
        self.get_logger().info(f'发送消息: counter = {self.counter}')

        # 计数器自增
        self.counter += 1

def main(args=None):
    """节点主函数"""
    # 初始化ROS2
    rclpy.init(args=args)

    # 创建节点实例
    publisher_node = PublisherNode()

    try:
        # 循环运行节点，处理回调函数
        # Ctrl+C会触发KeyboardInterrupt
        rclpy.spin(publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
