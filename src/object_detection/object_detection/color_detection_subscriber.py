import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ColorDetectionSubscriber(Node):
    def __init__(self):
        super().__init__('color_detection_subscriber')
        
        # 订阅 color_detection 话题
        self.subscription = self.create_subscription(
            String,                     # 消息类型
            'color_detection',          # 话题名称
            self.listener_callback,     # 回调函数
            10                          # 队列大小
        )
        self.subscription  # 防止 GC 释放

    def listener_callback(self, msg):
        # 在终端输出接收到的消息
        self.get_logger().info(f'Received color detection: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectionSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Subscriber shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
