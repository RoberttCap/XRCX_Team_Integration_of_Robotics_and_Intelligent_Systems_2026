import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import transforms3d
import numpy as np


class PuzzlebotPublisher(Node):
    
    def __init__(self):
        super().__init__('puzzlebot_publisher')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        self.publish_static_map_odom()

        self.t = 0.0          
        self.wheel_angle = 0.0  

        self.radius = 0.5   
        self.omega = 0.5    

        
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.timer_cb)

        self.get_logger().info('Puzzlebot Publisher iniciado.')

    
    
    def publish_static_map_odom(self):
        tf_msg = TransformStamped()
    
        tf_msg.header.stamp = RclpyTime(seconds=0).to_msg()
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id = 'odom'
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = 0.0
        tf_msg.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(tf_msg)
        self.get_logger().info('TF estatico map->odom publicado.')

    
    
    def timer_cb(self):
        self.t += self.dt

        x = self.radius * math.cos(self.omega * self.t)
        y = self.radius * math.sin(self.omega * self.t)

        
        yaw = self.omega * self.t + math.pi / 2.0

        
        v = self.radius * self.omega
        wheel_radius = 0.05  
        self.wheel_angle += (v / wheel_radius) * self.dt

      
        self.publish_odom_to_base_footprint(x, y, yaw)

    
        self.publish_joint_states()

    
   
    
    def publish_odom_to_base_footprint(self, x, y, yaw):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'

        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0

       
        
        q = transforms3d.euler.euler2quat(0.0, 0.0, yaw)
        tf_msg.transform.rotation.w = q[0]
        tf_msg.transform.rotation.x = q[1]
        tf_msg.transform.rotation.y = q[2]
        tf_msg.transform.rotation.z = q[3]

        self.tf_broadcaster.sendTransform(tf_msg)

    
    
    
    def publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['wheel_r_joint', 'wheel_l_joint']
        js.position = [self.wheel_angle, self.wheel_angle]
        js.velocity = []
        js.effort = []
        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()