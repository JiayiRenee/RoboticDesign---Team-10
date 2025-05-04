import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from rclpy.task import Future
from std_srvs.srv import Trigger



class MoveGroupCommanderInterfacesTesterNode(Node):
    """A ROS2 Node that publishes to a topics and will eventually call a service"""

    def __init__(self):
        super().__init__('move_group_commander_interfaces_tester_node')

        self.test_block_Pose_publisher = self.create_publisher(
            msg_type=Pose,
            topic='/arm_control/goal_pose',
            qos_profile=1)


        self.future: Future = None

        timer_period: float = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.print_count: int = 0


    def timer_callback(self):
        published_pose = Pose()
        published_pose.position.x = 0.3 ## limit the max pose to 4.5 
        published_pose.position.y = 0.0
        published_pose.position.z = 0.03
        self.test_block_Pose_publisher.publish(published_pose)



def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        interfaces_tester = MoveGroupCommanderInterfacesTesterNode()

        rclpy.spin(interfaces_tester)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()