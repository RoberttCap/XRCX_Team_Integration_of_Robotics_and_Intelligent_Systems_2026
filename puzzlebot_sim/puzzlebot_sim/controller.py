import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)

        self.waypoints = [
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 1.0),
            (0.0, 0.0)
        ]
        self.current_waypoint_index = 0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.has_odom = False

        self.kp_linear = 0.7
        self.kp_angular = 2.0
        self.max_linear = 0.5
        self.max_angular = 1.5
        self.position_tolerance = 0.05
        self.angle_tolerance = 0.15

        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.desired_x = []
        self.desired_y = []
        self.actual_x = []
        self.actual_y = []

        self.discretize_path()

        self.get_logger().info('Controller Node started.')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_theta = math.atan2(2.0 * qz * qw, 1.0 - 2.0 * qz * qz)
        self.has_odom = True

    def discretize_path(self):
        step = 0.02
        for i in range(len(self.waypoints)):
            start = self.waypoints[i]
            end = self.waypoints[(i + 1) % len(self.waypoints)]
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            dist = math.hypot(dx, dy)
            steps = max(1, int(dist / step))
            for j in range(steps + 1):
                t = j / steps
                self.desired_x.append(start[0] + dx * t)
                self.desired_y.append(start[1] + dy * t)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def control_loop(self):
        if not self.has_odom:
            return

        if self.current_waypoint_index >= len(self.waypoints):
            self.cmd_vel_pub.publish(Twist())
            return

        target_x, target_y = self.waypoints[self.current_waypoint_index]
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.current_theta)

        twist = Twist()
        if distance > self.position_tolerance:
            if abs(angle_error) > self.angle_tolerance:
                twist.linear.x = 0.0
            else:
                twist.linear.x = min(self.max_linear, self.kp_linear * distance)
            twist.angular.z = max(-self.max_angular,
                                  min(self.max_angular, self.kp_angular * angle_error))
        else:
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().info('Reached final waypoint. Stopping controller.')
                twist = Twist()
            else:
                self.get_logger().info(
                    f'Reached waypoint {self.current_waypoint_index}, moving to next.')
                twist = Twist()

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
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
