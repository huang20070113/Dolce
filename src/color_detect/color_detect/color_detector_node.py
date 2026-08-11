import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector_node')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/camera/image_raw', self.callback, 10)

    def callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 蓝色色块
        blue_mask = cv2.inRange(
            hsv, np.array([90, 80, 80]), np.array([130, 255, 255])
        )

        # 红色色块：Hue 在 HSV 中跨越 0，所以需要两段阈值
        red1 = cv2.inRange(
            hsv, np.array([0, 80, 80]), np.array([10, 255, 255])
        )
        red2 = cv2.inRange(
            hsv, np.array([170, 80, 80]), np.array([180, 255, 255])
        )
        red_mask = cv2.bitwise_or(red1, red2)

        result = image.copy()
        for mask, name, color in [
            (blue_mask, 'Blue', (255, 0, 0)),
            (red_mask, 'Red', (0, 0, 255)),
        ]:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                if cv2.contourArea(contour) > 500:
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(result, (x, y), (x + w, y + h), color, 2)
                    cv2.putText(result, name, (x, y - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        threshold = cv2.bitwise_or(blue_mask, red_mask)
        cv2.imshow('Color Recognition', result)
        cv2.imshow('Threshold', threshold)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = ColorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
