
import math

import rclpy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class Control(Node):

    def __init__(self):
        super().__init__('control')

        # Controller gains and actuator limits.
        self.kv = 0.8
        self.kw = 2.0
        self.v_max = 0.25
        self.w_max = 1.5
        self.goal_tolerance = 0.05
        self.orientation_tolerance = math.radians(2.0)
        self.heading_stop_threshold = 0.3

        # Latest robot pose from odometry.
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Goal pose commanded from another ROS 2 node or the CLI.
        self.x_goal = 0.0
        self.y_goal = 0.0
        self.theta_goal = 0.0

        self.odom_received = False
        self.goal_received = False
        self.goal_reached = False

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Pose2D, '/goal_pose', self.goal_callback, 10)

        # Run the controller at 50 Hz.
        self.timer = self.create_timer(0.02, self.control_callback)

        self.get_logger().info(
            'Waiting for a goal on /goal_pose (geometry_msgs/msg/Pose2D).'
        )

    def odom_callback(self, msg: Odometry):
        #Extract x, y and yaw from the odometry message.
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

    def goal_callback(self, msg: Pose2D):
        #Store the latest commanded goal pose.
        self.x_goal = msg.x
        self.y_goal = msg.y
        self.theta_goal = self.normalize_angle(msg.theta)
        self.goal_received = True
        self.goal_reached = False

        self.get_logger().info(
            'New goal -> x: %.2f m, y: %.2f m, theta: %.1f deg'
            % (
                self.x_goal,
                self.y_goal,
                math.degrees(self.theta_goal),
            )
        )

    def normalize_angle(self, angle: float) -> float:
        #Wrap an angle to the interval [-pi, pi].
        return math.atan2(math.sin(angle), math.cos(angle))

    def saturate(self, value: float, limit: float) -> float:
        #Clamp a scalar command to its symmetric limit.
        return max(-limit, min(limit, value))

    def publish_stop(self):
        #Publish a zero velocity command.
        self.cmd_vel_pub.publish(Twist())

    def control_callback(self):
        #Compute and publish the proportional control action.
        if not self.odom_received:
            return

        if not self.goal_received:
            self.publish_stop()
            return

        dx = self.x_goal - self.x
        dy = self.y_goal - self.y

        distance_error = math.sqrt(dx * dx + dy * dy)
        final_heading_error = self.normalize_angle(self.theta_goal - self.theta)

        cmd_msg = Twist()

        if distance_error >= self.goal_tolerance:
            desired_heading = math.atan2(dy, dx)
            raw_heading_error = desired_heading - self.theta
            heading_error = self.normalize_angle(raw_heading_error)

            # The cosine term suppresses forward motion when the robot is
            # pointing away from the goal, which produces cleaner trajectories.
            linear_cmd = self.kv * distance_error * math.cos(heading_error)
            angular_cmd = self.kw * heading_error

            if abs(heading_error) > self.heading_stop_threshold:
                linear_cmd = 0.0

            # Reduce the linear speed close to the goal to soften the approach
            # and reduce the residual position error.
            if distance_error < 0.2:
                linear_cmd *= 0.5

            cmd_msg.linear.x = self.saturate(linear_cmd, self.v_max)
            cmd_msg.angular.z = self.saturate(angular_cmd, self.w_max)
            self.goal_reached = False
        elif abs(final_heading_error) >= self.orientation_tolerance:
            # Once the robot reaches the target position, align to the
            # requested final heading in place.
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = self.saturate(
                self.kw * final_heading_error,
                self.w_max,
            )
            self.goal_reached = False
        else:
            if not self.goal_reached:
                self.get_logger().info('Goal reached. Stopping the robot.')
            self.goal_reached = True

        self.cmd_vel_pub.publish(cmd_msg)


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
