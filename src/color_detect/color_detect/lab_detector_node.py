import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class LabColorDetector(Node):
    def __init__(self):
        super().__init__('lab_detector_node')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # 目标颜色，OpenCV 顺序为 B、G、R
        # 蓝色色块实际偏青蓝色
        self.blue_bgr = np.array(
            [210, 170, 20],
            dtype=np.uint8
        )

        # 红色色块
        self.red_bgr = np.array(
            [0, 0, 255],
            dtype=np.uint8
        )

        # 红色和蓝色分别使用不同的 Lab 距离阈值
        # 数值越大越宽松，越小越严格
        self.blue_distance_threshold = 100
        self.red_distance_threshold = 20

        # 过滤小面积误识别
        self.min_area = 3000

        # 将目标颜色转换为 Lab
        self.blue_lab = self.bgr_to_lab(
            self.blue_bgr
        )
        self.red_lab = self.bgr_to_lab(
            self.red_bgr
        )

        self.get_logger().info(
            'Lab color distance detector started'
        )

    @staticmethod
    def bgr_to_lab(bgr_color):
        """
        将一个 BGR 颜色转换成 Lab 颜色。
        """
        color_image = np.zeros(
            (1, 1, 3),
            dtype=np.uint8
        )

        color_image[0, 0] = bgr_color

        lab_image = cv2.cvtColor(
            color_image,
            cv2.COLOR_BGR2LAB
        )

        # 转成 float32，防止距离计算溢出
        return lab_image[0, 0].astype(np.float32)

    @staticmethod
    def color_distance(lab_image, target_lab):
        """
        计算图像中每个像素与目标 Lab 颜色的欧氏距离。
        """
        lab_float = lab_image.astype(np.float32)

        target = target_lab.reshape(
            1,
            1,
            3
        )

        difference = lab_float - target

        distance = np.sqrt(
            np.sum(
                difference ** 2,
                axis=2
            )
        )

        return distance

    @staticmethod
    def clean_mask(mask):
        """
        使用形态学操作去除小噪声并填补色块空洞。
        """
        open_kernel = np.ones(
            (5, 5),
            dtype=np.uint8
        )

        close_kernel = np.ones(
            (11, 11),
            dtype=np.uint8
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            open_kernel
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            close_kernel
        )

        return mask

    def draw_largest_color_area(
        self,
        result,
        mask,
        name,
        draw_color
    ):
        """
        查找并绘制某种颜色面积最大的区域。
        """
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return

        # 每种颜色只保留最大轮廓
        largest_contour = max(
            contours,
            key=cv2.contourArea
        )

        area = cv2.contourArea(
            largest_contour
        )

        if area < self.min_area:
            return

        x, y, w, h = cv2.boundingRect(
            largest_contour
        )

        center_x = x + w // 2
        center_y = y + h // 2

        # 绘制外接矩形
        cv2.rectangle(
            result,
            (x, y),
            (x + w, y + h),
            draw_color,
            3
        )

        # 绘制中心点
        cv2.circle(
            result,
            (center_x, center_y),
            7,
            draw_color,
            -1
        )

        label = (
            f'{name} '
            f'({center_x}, {center_y})'
        )

        cv2.putText(
            result,
            label,
            (x, max(y - 12, 30)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            draw_color,
            2
        )

    def image_callback(self, msg):
        try:
            # ROS 图像消息转成 OpenCV BGR 图像
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as error:
            self.get_logger().error(
                f'Image conversion failed: {error}'
            )
            return

        # BGR 转 Lab
        lab_image = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2LAB
        )

        # 分别计算蓝色和红色的 Lab 距离
        blue_distance = self.color_distance(
            lab_image,
            self.blue_lab
        )

        red_distance = self.color_distance(
            lab_image,
            self.red_lab
        )

        # 生成蓝色二值掩膜
        blue_mask = (
            blue_distance <
            self.blue_distance_threshold
        ).astype(np.uint8) * 255

        # 生成红色二值掩膜
        red_mask = (
            red_distance <
            self.red_distance_threshold
        ).astype(np.uint8) * 255

        # 去除噪声
        blue_mask = self.clean_mask(
            blue_mask
        )
        red_mask = self.clean_mask(
            red_mask
        )

        # 合并红色和蓝色二值图
        threshold = cv2.bitwise_or(
            blue_mask,
            red_mask
        )

        result = image.copy()

        # 绘制蓝色色块
        self.draw_largest_color_area(
            result,
            blue_mask,
            'Blue',
            (255, 0, 0)
        )

        # 绘制红色色块
        self.draw_largest_color_area(
            result,
            red_mask,
            'Red',
            (0, 0, 255)
        )

        # 显示识别结果
        cv2.imshow(
            'Lab Color Recognition',
            result
        )

        # 显示最终二值图
        cv2.imshow(
            'Lab Threshold',
            threshold
        )

        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = LabColorDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()