
## 🔧 Requirements

- ROS 2 Humble
- Python 3.10+
- Intel RealSense camera (e.g., D435i)
- Dependencies:
  - OpenCV
  - numpy
  - pyrealsense2
  - sensor_msgs, std_msgs, geometry_msgs, cv_bridge

## 🚀 Running the Node

You can run the detection node directly (without launch file):
colcon build
source
```bash
ros2 run object_detection object_detection

Topic Name	Message Type	Description
/color_detection	std_msgs/String	Text-based log of detected objects
/color_image	sensor_msgs/Image	Annotated image stream
/block_pose	geometry_msgs/Pose	3D position of detected blocks
/bin_pose	geometry_msgs/Pose	3D position of detected bins

Integration with Robotic Arm
Your robotic arm node can subscribe to:

/block_pose: For pick-up targets

/bin_pose: For placing into the correct bin





import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math

class RealSenseColorDetectionNode(Node):
    def __init__(self):
        super().__init__('realsense_color_detection')
        self.publisher_ = self.create_publisher(String, 'color_detection', 10)
        self.image_publisher_ = self.create_publisher(Image, 'color_image', 10)
        self.block_publisher = self.create_publisher(Pose, 'block_pose', 10)
        self.bin_publisher = self.create_publisher(Pose, 'bin_pose', 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.bridge = CvBridge()

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        self.fx, self.fy = 615.0, 615.0
        self.cx, self.cy = 320, 240

        self.blocks = {}
        self.bins = {}

        # Manually defined offsets (meters)
        self.offset_x = 0.02  # move 2cm in x
        self.offset_y = 0.01  # move 1cm in y
        self.offset_z = 0.00  # no change in z

        self.timer = self.create_timer(0.1, self.process_frames)

    def process_frames(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            self.get_logger().warn("No depth or color frame available.")
            return

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        color_image = cv2.GaussianBlur(color_image, (5, 5), 0)
        hsv_img = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        COLOR_RANGES = {
            'red': (np.array([0, 80, 50]), np.array([10, 255, 255])),
            'yellow': (np.array([20, 150, 100]), np.array([35, 255, 255]))
        }

        COLOR_MAP = {
            'red': (0, 0, 255),
            'yellow': (0, 255, 255)
        }

        for color, (lower, upper) in COLOR_RANGES.items():
            mask = cv2.inRange(hsv_img, lower, upper)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                center_x, center_y = int(rect[0][0]), int(rect[0][1])

                depth_values = depth_image[center_y-5:center_y+5, center_x-5:center_x+5].astype(float)
                depth_values = depth_values[depth_values > 0]
                if len(depth_values) == 0:
                    continue
                depth_value = np.median(depth_values) / 1000.0

                X, Y, Z = self.pixel_to_world(center_x, center_y, depth_value)

                # Add manual offset
                X += self.offset_x
                Y += self.offset_y
                Z += self.offset_z

                if area < 5000 and Z < 0.4:
                    self.detect_blocks(color, X, Y, Z)
                elif area > 5000 and Z > 0.5:
                    self.detect_bins(color, X, Y, Z)

                cv2.drawContours(color_image, [box], 0, COLOR_MAP[color], 2)
                cv2.putText(color_image, f"{color}: ({X:.2f}, {Y:.2f}, {Z:.2f})m",
                            (center_x, center_y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_MAP[color], 2)

        img_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        self.image_publisher_.publish(img_msg)
        
        cv2.imshow("Color Detection", color_image)
        cv2.waitKey(1)

    def detect_blocks(self, color, X, Y, Z):
        if color not in self.blocks:
            self.blocks[color] = []
        self.blocks[color].append((X, Y, Z))

        self.publish_task(self.block_publisher, X, Y, Z, color)
        self.broadcast_tf(X, Y, Z, f"block_{color}")
        self.get_logger().info(f"[BLOCK] {color} at ({X:.2f}, {Y:.2f}, {Z:.2f})")

    def detect_bins(self, color, X, Y, Z):
        self.bins[color] = (X, Y, Z)
        self.publish_task(self.bin_publisher, X, Y, Z, color)
        self.broadcast_tf(X, Y, Z, f"bin_{color}")
        self.get_logger().info(f"[BIN] {color} at ({X:.2f}, {Y:.2f}, {Z:.2f})")

    def publish_task(self, publisher, X, Y, Z, color):
        msg = Pose()
        msg.position.x = X
        msg.position.y = Y
        msg.position.z = Z
        msg.orientation.w = 1.0
        publisher.publish(msg)

    def broadcast_tf(self, X, Y, Z, child_frame_id):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "camera_color_optical_frame"  # Base frame
        t.child_frame_id = child_frame_id  # Object frame name
        t.transform.translation.x = X
        t.transform.translation.y = Y
        t.transform.translation.z = Z
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

    def pixel_to_world(self, u, v, depth):
        X = (u - self.cx) * depth / self.fx
        Y = (v - self.cy) * depth / self.fy
        Z = depth
        return X, Y, Z

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
