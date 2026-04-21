
import math
import rclpy
from nav_msgs.msg import Odometry
from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import Float32


class Localisation(Node):

    def __init__(self):
        super().__init__('localisation')

        # Robot geometry.
        self.wheel_radius = 0.05
        self.wheel_base = 0.19

        # Estimated robot state.
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Measured wheel angular speeds.
        self.wr = 0.0
        self.wl = 0.0

        self.last_time = self.get_clock().now()
        self.odom_msg = Odometry()

        # Wheel speeds behave like sensor signals, so use sensor-data QoS.
        self.create_subscription(
            Float32,
            '/wr',
            self.wr_callback,
            qos.qos_profile_sensor_data,
        )
        self.create_subscription(
            Float32,
            '/wl',
            self.wl_callback,
            qos.qos_profile_sensor_data,
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)

    def wr_callback(self, msg: Float32):
        # Store the latest right wheel speed.
        self.wr = msg.data

    def wl_callback(self, msg: Float32):
        # Store the latest left wheel speed.
        self.wl = msg.data

    def get_robot_vel(self):
        # Linear and angular velocity from wheel speeds.
        self.linear_velocity = (self.wheel_radius * (self.wr + self.wl)) / 2.0
        self.angular_velocity = (self.wheel_radius * (self.wr - self.wl)) / self.wheel_base

    def update_pose(self):
        # Integrate the dead-reckoning model using the current velocity.
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        self.x += self.linear_velocity * math.cos(self.yaw) * dt
        self.y += self.linear_velocity * math.sin(self.yaw) * dt
        self.yaw += self.angular_velocity * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        return current_time

    def fill_odom_message(self, current_time):
        # Create a new Odometry msg
        odom_msg = Odometry()
        # Fill with robot pose
        # Get current time
        odom_msg.header.stamp = current_time.to_msg()
        # Use standard TF frame names without a leading slash.
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        # x, y, z positions (m)
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Convert planar yaw into a quaternion without external deps.
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.angular.z = self.angular_velocity

        self.odom_msg = odom_msg

    def timer_callback(self):
        # Update the pose estimate and publish the odometry message.
        self.get_robot_vel()
        current_time = self.update_pose()
        self.fill_odom_message(current_time)
        self.odom_pub.publish(self.odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Localisation()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
