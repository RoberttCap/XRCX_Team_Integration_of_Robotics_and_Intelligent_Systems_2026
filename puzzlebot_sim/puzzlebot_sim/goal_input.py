
import math
import threading

import rclpy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from rclpy.node import Node


class GoalInput(Node):
    #Read absolute x, y and heading goals from the terminal.

    def __init__(self):
        super().__init__('goal_input')

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.odom_received = False
        self.pose_lock = threading.Lock()

        self.goal_tolerance = 0.05
        self.orientation_tolerance = math.radians(2.0)
        self.default_side_length = 1.0

        self.sequence_lock = threading.Lock()
        self.sequence_active = False
        self.sequence_waypoints = []
        self.sequence_index = 0

        self.goal_pub = self.create_publisher(Pose2D, '/goal_pose', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sequence_timer = self.create_timer(0.05, self.sequence_callback)

        self.input_thread = threading.Thread(
            target=self.input_loop,
            daemon=True,
        )
        self.input_thread.start()

    def odom_callback(self, msg: Odometry):
        #Store the latest robot pose for figure generation and sequencing.
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        siny_cosp = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )

        with self.pose_lock:
            self.x = position.x
            self.y = position.y
            self.theta = math.atan2(siny_cosp, cosy_cosp)
            self.odom_received = True

    def normalize_angle(self, angle: float) -> float:
        #Wrap an angle to the interval [-pi, pi].
        return math.atan2(math.sin(angle), math.cos(angle))

    def publish_goal(self, x: float, y: float, theta: float):
        #Publish an absolute goal pose.
        goal_msg = Pose2D()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.theta = self.normalize_angle(theta)
        self.goal_pub.publish(goal_msg)

    def publish_waypoint(self, waypoint: Pose2D):
        #Publish a figure waypoint and print it to the terminal.
        self.publish_goal(waypoint.x, waypoint.y, waypoint.theta)
        print(
            'Published waypoint %d/%d -> x: %.2f, y: %.2f, theta: %.1f deg'
            % (
                self.sequence_index + 1,
                len(self.sequence_waypoints),
                waypoint.x,
                waypoint.y,
                math.degrees(waypoint.theta),
            )
        )

    def build_polygon_waypoints(self, sides: int, side_length: float):
        #Create a closed polygon path from the robot's current pose.
        with self.pose_lock:
            x = self.x
            y = self.y
            theta = self.theta

        start_theta = theta
        turn_angle = 2.0 * math.pi / sides
        waypoints = []

        for edge_index in range(sides):
            x += side_length * math.cos(theta)
            y += side_length * math.sin(theta)

            if edge_index == sides - 1:
                goal_theta = start_theta
            else:
                goal_theta = self.normalize_angle(theta + turn_angle)

            waypoint = Pose2D()
            waypoint.x = x
            waypoint.y = y
            waypoint.theta = goal_theta
            waypoints.append(waypoint)

            theta = goal_theta

        return waypoints

    def goal_reached(self, waypoint: Pose2D) -> bool:
        #Check whether the robot has reached a waypoint.
        with self.pose_lock:
            dx = waypoint.x - self.x
            dy = waypoint.y - self.y
            heading_error = self.normalize_angle(waypoint.theta - self.theta)

        distance_error = math.sqrt(dx * dx + dy * dy)
        return (
            distance_error < self.goal_tolerance
            and abs(heading_error) < self.orientation_tolerance
        )

    def start_polygon_sequence(self, sides: int, side_length: float, name: str):
        #Generate and start a square or pentagon path.
        if not self.odom_received:
            print('Waiting for /odom before starting a figure.')
            return

        if side_length <= 0.0:
            print('Side length must be greater than zero.')
            return

        with self.sequence_lock:
            self.sequence_waypoints = self.build_polygon_waypoints(
                sides,
                side_length,
            )
            self.sequence_index = 0
            self.sequence_active = True

        print(
            'Starting %s with side length %.2f m from the current robot pose.'
            % (name, side_length)
        )
        self.publish_waypoint(self.sequence_waypoints[self.sequence_index])

    def sequence_callback(self):
        """Advance to the next waypoint once the current one is reached."""
        if not self.odom_received:
            return

        with self.sequence_lock:
            if not self.sequence_active:
                return
            current_waypoint = self.sequence_waypoints[self.sequence_index]

        if not self.goal_reached(current_waypoint):
            return

        with self.sequence_lock:
            self.sequence_index += 1
            if self.sequence_index >= len(self.sequence_waypoints):
                self.sequence_active = False
                self.sequence_waypoints = []
                print('Figure completed.')
                return

            next_waypoint = self.sequence_waypoints[self.sequence_index]

        self.publish_waypoint(next_waypoint)

    def input_loop(self):
        #Prompt for absolute goals in the odom/world frame.
        print('Enter goal as: x y theta_deg')
        print('Example: 1.0 1.0 90')
        print('This is an absolute pose, not a relative motion command.')
        print('You can also type: square [side_m] or pentagon [side_m]')
        print("Type 'q' to quit.")

        while rclpy.ok():
            try:
                user_input = input('goal > ').strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not user_input:
                continue

            if user_input.lower() in {'q', 'quit', 'exit'}:
                break

            parts = user_input.split()
            command = parts[0].lower()

            if command in {'square', 'pentagon', 'pentagone'}:
                side_length = self.default_side_length
                if len(parts) == 2:
                    try:
                        side_length = float(parts[1])
                    except ValueError:
                        print('Invalid side length. Example: square 1.0')
                        continue
                elif len(parts) != 1:
                    print('Use: square [side_m] or pentagon [side_m]')
                    continue

                sides = 4 if command == 'square' else 5
                figure_name = 'square' if sides == 4 else 'pentagon'
                self.start_polygon_sequence(sides, side_length, figure_name)
                continue

            if len(parts) != 3:
                print('Enter exactly: x y theta_deg')
                continue

            try:
                x = float(parts[0])
                y = float(parts[1])
                theta_deg = float(parts[2])
            except ValueError:
                print('Invalid input')
                continue

            theta = math.atan2(
                math.sin(math.radians(theta_deg)),
                math.cos(math.radians(theta_deg)),
            )

            with self.sequence_lock:
                self.sequence_active = False
                self.sequence_waypoints = []
                self.sequence_index = 0
            self.publish_goal(x, y, theta)

            print(
                f'Published goal -> x: {x:.2f}, y: {y:.2f}, theta: {theta_deg:.1f} deg'
            )

        self.get_logger().info('Goal input stopped.')
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = GoalInput()

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
