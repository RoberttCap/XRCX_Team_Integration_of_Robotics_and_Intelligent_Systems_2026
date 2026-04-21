import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class DifferentialDriveNode(Node):
    def __init__(self):
        super().__init__('differential_drive_node')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        self.pos_pub = self.create_publisher(PoseStamped, 'pos_sim', 10)
        self.wheel_vel_pub = self.create_publisher(Float64MultiArray, 'wheel_velocities', 10)

        self.wheel_base = 0.19
        self.wheel_radius = 0.05

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.latest_v = 0.0
        self.latest_omega = 0.0

        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info('Differential Drive Node started.')

    def cmd_vel_callback(self, msg):
        self.latest_v = msg.linear.x
        self.latest_omega = msg.angular.z

    def timer_callback(self):
        v = self.latest_v
        omega = self.latest_omega

        omega_r = (v + omega * self.wheel_base / 2.0) / self.wheel_radius
        omega_l = (v - omega * self.wheel_base / 2.0) / self.wheel_radius

        self.x += v * math.cos(self.theta) * self.dt
        self.y += v * math.sin(self.theta) * self.dt
        self.theta += omega * self.dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        pos_msg = PoseStamped()
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        pos_msg.header.frame_id = 'odom'
        pos_msg.pose.position.x = self.x
        pos_msg.pose.position.y = self.y
        pos_msg.pose.position.z = 0.0
        pos_msg.pose.orientation.z = math.sin(self.theta / 2.0)
        pos_msg.pose.orientation.w = math.cos(self.theta / 2.0)
        self.pos_pub.publish(pos_msg)

        self.broadcast_odom_tf()

        wheel_vel_msg = Float64MultiArray()
        wheel_vel_msg.data = [omega_r, omega_l]
        self.wheel_vel_pub.publish(wheel_vel_msg)

    def broadcast_odom_tf(self):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = math.sin(self.theta / 2.0)
        tf_msg.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDriveNode()
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
