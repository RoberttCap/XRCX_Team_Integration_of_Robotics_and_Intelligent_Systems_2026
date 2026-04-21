import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class PuzzlebotPublisher(Node):
    
    def __init__(self):
        super().__init__('puzzlebot_publisher')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        self.subscription = self.create_subscription(
            Float64MultiArray,
            'wheel_velocities',
            self.wheel_vel_callback,
            10)

        self.publish_static_map_odom()

        self.t = 0.0          
        self.wheel_angle_r = 0.0  
        self.wheel_angle_l = 0.0  

        self.radius = 0.5   
        self.omega = 0.5    
        self.wheel_base = 0.19  # distance between wheels
        self.wheel_radius = 0.05

        
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

    def wheel_vel_callback(self, msg):
        if len(msg.data) >= 2:
            omega_r = msg.data[0]
            omega_l = msg.data[1]
            self.wheel_angle_r += omega_r * self.dt
            self.wheel_angle_l += omega_l * self.dt

    def timer_cb(self):
        self.t += self.dt

        self.publish_joint_states()

    
    
    
    def publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['wheel_r_joint', 'wheel_l_joint']
        js.position = [self.wheel_angle_r, self.wheel_angle_l]
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
        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()