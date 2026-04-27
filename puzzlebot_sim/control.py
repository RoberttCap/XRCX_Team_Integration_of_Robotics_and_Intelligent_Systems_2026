import math

import rclpy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class Control(Node):

    def __init__(self):
        super().__init__('control')

        self.declare_parameter('kv', 0.8)
        self.declare_parameter('kw', 2.0)
        self.declare_parameter('v_max', 0.25)
        self.declare_parameter('w_max', 1.5)
        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('orientation_tolerance_deg', 2.0)

        self.kv = self.get_parameter('kv').value
        self.kw = self.get_parameter('kw').value
        self.v_max = self.get_parameter('v_max').value
        self.w_max = self.get_parameter('w_max').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.orientation_tolerance = math.radians(
            self.get_parameter('orientation_tolerance_deg').value
        )

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.x_goal = 0.0
        self.y_goal = 0.0
        self.theta_goal = 0.0

        self.odom_received = False
        self.goal_received = False
        self.goal_reached = False

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(Pose2D, 'goal_pose', self.goal_callback, 10)

        self.timer = self.create_timer(0.02, self.control_callback)

        self.get_logger().info('Waiting for goal_pose...')

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.x = position.x
        self.y = position.y

        siny_cosp = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )

        self.theta = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

    def goal_callback(self, msg):
        self.x_goal = msg.x
        self.y_goal = msg.y
        self.theta_goal = self.normalize_angle(msg.theta)

        self.goal_received = True
        self.goal_reached = False

        self.get_logger().info(
            'New goal: x=%.2f y=%.2f theta=%.1f'
            % (
                self.x_goal,
                self.y_goal,
                math.degrees(self.theta_goal),
            )
        )

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def saturate(self, value, limit):
        return max(-limit, min(limit, value))

    def publish_stop(self):
        self.cmd_vel_pub.publish(Twist())

    def control_callback(self):
        if not self.odom_received:
            return

        if not self.goal_received:
            self.publish_stop()
            return

        dx = self.x_goal - self.x
        dy = self.y_goal - self.y

        distance_error = math.sqrt(dx * dx + dy * dy)
        final_heading_error = self.normalize_angle(
            self.theta_goal - self.theta
        )

        cmd = Twist()

        if distance_error >= self.goal_tolerance:
            desired_heading = math.atan2(dy, dx)
            heading_error = self.normalize_angle(
                desired_heading - self.theta
            )

            linear_cmd = self.kv * distance_error * math.cos(heading_error)
            angular_cmd = self.kw * heading_error

            cmd.linear.x = self.saturate(linear_cmd, self.v_max)
            cmd.angular.z = self.saturate(angular_cmd, self.w_max)

            self.goal_reached = False

        elif abs(final_heading_error) >= self.orientation_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = self.saturate(
                self.kw * final_heading_error,
                self.w_max,
            )

            self.goal_reached = False

        else:
            if not self.goal_reached:
                self.get_logger().info('Goal reached')

            self.goal_reached = True
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Control()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()