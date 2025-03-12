import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

class RealSenseColorDetectionNode(Node):
    def __init__(self):
        super().__init__('realsense_color_detection')
        self.publisher_ = self.create_publisher(String, 'color_detection', 10)
        self.image_publisher_ = self.create_publisher(Image, 'color_image', 10)
        self.bridge = CvBridge()
        
        # 初始化 RealSense 相机
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        
        self.timer = self.create_timer(0.1, self.process_frames)

    def process_frames(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # ✅ 先进行高斯模糊，减少噪声干扰
        color_image = cv2.GaussianBlur(color_image, (5, 5), 0)

        # 转换到 HSV 颜色空间
        hsv_img = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # ✅ 调整光线鲁棒性，增加紫色和粉色的检测
        COLOR_RANGES = {
            'red':    (np.array([0, 100, 50]), np.array([10, 255, 255])),
            'blue':   (np.array([100, 120, 50]), np.array([140, 255, 255])),
            'green':  (np.array([35, 50, 50]), np.array([85, 255, 255])),
            'yellow': (np.array([20, 100, 100]), np.array([35, 255, 255])),
            'purple': (np.array([130, 50, 50]), np.array([160, 255, 255])),  # 紫色
            'pink':   (np.array([160, 50, 50]), np.array([180, 255, 255]))   # 粉色
        }

        COLOR_MAP = {
            'red': (0, 0, 255), 
            'blue': (255, 0, 0), 
            'green': (0, 255, 0),
            'yellow': (0, 255, 255),
            'purple': (128, 0, 128),  # 紫色
            'pink': (255, 20, 147)    # 粉色
        }

        detection_result = []
        for color, (lower, upper) in COLOR_RANGES.items():
            mask = cv2.inRange(hsv_img, lower, upper)

            # ✅ 形态学操作，去除小噪声
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                if cv2.contourArea(cnt) < 800:  # ✅ 提高面积阈值，减少误检
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                center_x, center_y = int(x + w / 2), int(y + h / 2)

                # ✅ 计算深度，避免超远距离噪声
                depth_value = depth_frame.get_distance(center_x, center_y)
                if depth_value > 3.0 or depth_value == 0.0:  # 过滤超远距离和错误测距
                    continue

                # 在图像上绘制矩形和标签
                cv2.rectangle(color_image, (x, y), (x + w, y + h), COLOR_MAP[color], 2)
                cv2.putText(color_image, f"{color}: {depth_value:.2f}m", (x, y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_MAP[color], 2)
                detection_result.append(f"{color} at ({center_x},{center_y}): {depth_value:.2f}m")

        # 发送检测结果到 ROS 话题
        if detection_result:
            msg = String()
            msg.data = " | ".join(detection_result)
            self.publisher_.publish(msg)
        
        # 发送图像话题
        img_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        self.image_publisher_.publish(img_msg)

        # ✅ 显示 OpenCV 窗口
        cv2.imshow("Color Detection", color_image)
        cv2.waitKey(1)
        
    def shutdown(self):
        self.pipeline.stop()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseColorDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node.')
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

