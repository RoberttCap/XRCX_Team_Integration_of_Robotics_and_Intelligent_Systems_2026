#!/usr/bin/env python3

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

        # Publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.robot_vel = Twist()

        # Shutdown handler: when Ctrl+C is pressed, stop the robot first
        signal.signal(signal.SIGINT, self.shutdown_function)

        self.lidar = LaserScan()
        self.received_scan = False

        self.d_safety = 0.3   # Safety distance threshold in meters
        self.get_kv = 0.5     # Linear velocity gain
        self.get_kw = 0.5     # Angular velocity gain

        timer_period = 0.05   # 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Node initialized!!!")

    def timer_callback(self):
        if self.received_scan:
            closest_range, theta_closest = self.get_closest_object()

            if np.isinf(closest_range):
                self.get_logger().warn("WARNING: No valid range readings received.")
            else:
                self.get_logger().info("Closest range: {}".format(closest_range))
                self.get_logger().info("Theta closest: {}".format(theta_closest))

                # Get the linear and angular speed
                d_diff = closest_range - self.d_safety
                v = self.get_kv * d_diff
                w = self.get_kw * theta_closest

                self.robot_vel.linear.x = v
                self.robot_vel.angular.z = w

                self.cmd_vel_pub.publish(self.robot_vel)
        else:
            self.get_logger().info("No scan received yet.")

    def get_closest_object(self):
        self.angle_min = self.lidar.angle_min
        self.angle_max = self.lidar.angle_max
        self.range_min = self.lidar.range_min

        closest_range = min(self.lidar.ranges)
        closest_index = self.lidar.ranges.index(closest_range)
        angle_increment = self.lidar.angle_increment

        # Limit theta_closest angle to be between -pi and pi
        theta_closest = self.angle_min + closest_index * angle_increment
        theta_closest = np.arctan2(np.sin(theta_closest), np.cos(theta_closest))

        return closest_range, theta_closest

    def lidar_cb(self, lidar_msg):
        # This function receives the ROS LaserScan message
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