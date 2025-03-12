import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray  # 用于发布目标中心点和距离

# 定义多种颜色的 HSV 范围
color_ranges = {
    "red": [(0, 120, 70), (10, 255, 255), (170, 120, 70), (180, 255, 255)],
    "green": [(40, 40, 40), (90, 255, 255)],
    "blue": [(100, 100, 40), (140, 255, 255)],
    "yellow": [(20, 100, 100), (35, 255, 255)],
    "purple": [(125, 50, 50), (160, 255, 255)],
    "pink": [(140, 50, 150), (170, 255, 255)]
}

# 颜色的 BGR 显示颜色（用于绘制框）
color_bgr = {
    "red": (0, 0, 255),
    "green": (0, 255, 0),
    "blue": (255, 0, 0),
    "yellow": (0, 255, 255),
    "purple": (128, 0, 128),
    "pink": (255, 192, 203)
}

class ImageDepthSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)

        # 订阅 RGB 和 深度图像
        self.sub_color = self.create_subscription(
            Image, '/camera/color/image_raw', self.listener_callback_color, 10
        )
        self.sub_depth = self.create_subscription(
            Image, '/camera/depth/image_rect_raw', self.listener_callback_depth, 10
        )

        # OpenCV 处理工具
        self.cv_bridge = CvBridge()
        self.depth_image = None  # 存储深度图像
        self.pub = self.create_publisher(Float32MultiArray, 'object_center_distance', 10)

    def listener_callback_color(self, data):
        """ 处理 RGB 图像，进行多颜色目标检测 """
        self.get_logger().info('Receiving color video frame')
        color_image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        self.object_detect(color_image)

    def listener_callback_depth(self, data):
        """ 订阅深度图像，用于计算目标距离 """
        self.get_logger().info('Receiving depth video frame')
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(data, '16UC1')

    def object_detect(self, image):
        """ 颜色检测，获取目标物体的坐标和深度 """
        if self.depth_image is None:
            return

        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        for color, ranges in color_ranges.items():
            # 生成颜色掩码
            if len(ranges) == 4:
                mask1 = cv2.inRange(hsv_img, np.array(ranges[0]), np.array(ranges[1]))
                mask2 = cv2.inRange(hsv_img, np.array(ranges[2]), np.array(ranges[3]))
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                mask = cv2.inRange(hsv_img, np.array(ranges[0]), np.array(ranges[1]))

            # 查找轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                if cv2.contourArea(cnt) < 500:  # 过滤小目标
                    continue

                (x, y, w, h) = cv2.boundingRect(cnt)
                center_x, center_y = int(x + w / 2), int(y + h / 2)

                # 获取深度信息
                distance_mm = self.depth_image[center_y, center_x].astype(float)
                
                # 过滤无效深度数据
                if distance_mm == 0 or np.isnan(distance_mm):
                    continue  # 跳过无效深度点

                # 单位转换 mm -> 米
                distance_meters = distance_mm / 1000.0

                # 发布 `{x, y, depth}`
                msg = Float32MultiArray()
                msg.data = [float(center_x), float(center_y), float(distance_meters)]
                self.pub.publish(msg)

                # 画出矩形框
                cv2.rectangle(image, (x, y), (x + w, y + h), color_bgr[color], 2)
                cv2.circle(image, (center_x, center_y), 5, color_bgr[color], -1)

                # 显示颜色名称和深度信息
                text = f"{color} {distance_meters:.2f}m"
                cv2.putText(image, text, (x, y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr[color], 2)

        # 显示检测结果
        cv2.imshow("Multi-Color Detection", image)
        cv2.waitKey(10)


def main(args=None):
    rclpy.init(args=args)
    node = ImageDepthSubscriber("image_depth_sub")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


