import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import numpy as np
import signal
import sys


class LaserScanSub(Node):
    def __init__(self):
        super().__init__('laser_scan_subscriber')

        self.sub = self.create_subscription(
            LaserScan,
            "scan",
            self.lidar_cb,
            10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.robot_vel = Twist()

        signal.signal(signal.SIGINT, self.shutdown_function)

        self.lidar = LaserScan()
        self.received_scan = False

        self.d_safety = 0.3
        self.dstart_ao = 1.0   # <- 1 metro

        self.v = 0.2           # velocidad lineal constante en modo normal/avoid
        self.kw = 0.8          # ganancia angular para evasión
        self.forward_w = 0.0   # giro normal cuando no hay obstáculo

        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Node initialized!!!")

    def timer_callback(self):
        if not self.received_scan:
            self.get_logger().info("No scan received yet.")
            return

        closest_range, theta_closest = self.get_closest_object()

        if closest_range is None:
            self.get_logger().warn("WARNING: No valid range readings received.")
            self.stop_robot()
            return

        self.get_logger().info(f"Closest range: {closest_range:.3f}")
        self.get_logger().info(f"Theta closest: {theta_closest:.3f}")

        self.obstacle_avoidance(closest_range, theta_closest)

    def get_closest_object(self):
        ranges = np.array(self.lidar.ranges, dtype=np.float32)

        valid_mask = np.isfinite(ranges)
        valid_mask &= ranges >= self.lidar.range_min
        valid_mask &= ranges <= self.lidar.range_max

        if not np.any(valid_mask):
            return None, None

        valid_indices = np.where(valid_mask)[0]
        valid_ranges = ranges[valid_mask]

        min_idx_local = np.argmin(valid_ranges)
        closest_range = valid_ranges[min_idx_local]
        closest_index = valid_indices[min_idx_local]

        theta_closest = self.lidar.angle_min + closest_index * self.lidar.angle_increment
        theta_closest = np.arctan2(np.sin(theta_closest), np.cos(theta_closest))

        return float(closest_range), float(theta_closest)

    def lidar_cb(self, lidar_msg):
        self.lidar = lidar_msg
        self.received_scan = True

    def stop_robot(self):
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)

    def shutdown_function(self, signum, frame):
        self.get_logger().info("Shutting down. Stopping robot...")
        self.stop_robot()

        if rclpy.ok():
            rclpy.shutdown()

        sys.exit(0)

    def goal_to_closest_object(self, closest_range, theta_closest):
        # Dirección opuesta al obstáculo
        theta_ao = theta_closest + np.pi
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        self.robot_vel.linear.x = self.v
        self.robot_vel.angular.z = self.kw * theta_ao

        return self.robot_vel

    def object_too_close(self, closest_range):
        return closest_range <= self.d_safety

    def object_behind(self, theta_closest):
        return abs(theta_closest) > (np.pi / 2)


    def obstacle_avoidance(self, closest_range, theta_closest):
        if self.object_too_close(closest_range):
            self.stop_robot()

        elif self.object_behind(theta_closest):
            self.robot_vel.linear.x = self.v
            self.robot_vel.angular.z = self.forward_w
            self.cmd_vel_pub.publish(self.robot_vel)

        elif closest_range < self.dstart_ao:
            vel = self.goal_to_closest_object(closest_range, theta_closest)
            self.cmd_vel_pub.publish(vel)

        else:
            self.robot_vel.linear.x = self.v
            self.robot_vel.angular.z = self.forward_w
            self.cmd_vel_pub.publish(self.robot_vel)

def main(args=None):
    rclpy.init(args=args)

    m_p = LaserScanSub()

    try:
        rclpy.spin(m_p)
    except KeyboardInterrupt:
        pass
    finally:
        m_p.stop_robot()
        m_p.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()