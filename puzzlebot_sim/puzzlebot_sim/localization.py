import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')

        self.subscription = self.create_subscription(
            PoseStamped,
            'pos_sim',
            self.pos_callback,
            10)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.get_logger().info('Localization Node started.')

    def pos_callback(self, msg):
        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose = msg.pose
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        self.odom_pub.publish(odom_msg)

        theta = math.atan2(
            2.0 * msg.pose.orientation.z * msg.pose.orientation.w,
            1.0 - 2.0 * msg.pose.orientation.z * msg.pose.orientation.z)
        self.get_logger().info(
            f'Position: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, theta={theta:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
