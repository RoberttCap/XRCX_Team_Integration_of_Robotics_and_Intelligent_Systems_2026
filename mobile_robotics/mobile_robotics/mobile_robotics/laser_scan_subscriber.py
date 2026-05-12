import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import LaserScan 

class LaserScanSub(Node): 
    def __init__(self): 
        super().__init__('laser_scan_subscriber') 
        self.sub = self.create_subscription(LaserScan, "scan", self.lidar_cb, 10) 
        self.lidar = LaserScan() # Data from lidar will be stored here. 
        timer_period = 1.0 
        self.timer = self.create_timer(timer_period, self.timer_callback) 
        self.get_logger().info("Node initialized!!!") 

    def timer_callback(self): 
        self.get_logger().info("Angle min: " + str(self.lidar.angle_min))
        self.get_logger().info("Angle max: " + str(self.lidar.angle_max))
        self.get_logger().info("Angle increment: " + str(self.lidar.angle_increment))
        self.get_logger().info("Range min: " + str(self.lidar.range_min))
        self.get_logger().info("Range max: " + str(self.lidar.range_max))
        self.get_logger().info("header.frame_id: " + str(self.lidar.header.frame_id))
        self.get_logger().info("Range readings: " + str(self.lidar.ranges[0]))
        self.get_logger().info("Last Intensity lecture: " + str(self.lidar.intensities[-1]))
        

    def lidar_cb(self, lidar_msg): 
        ## This function receives the ROS LaserScan message 
        self.lidar =  lidar_msg  

def main(args=None): 
    rclpy.init(args=args) 
    m_p=LaserScanSub() 
    rclpy.spin(m_p) 
    m_p.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 

    main() 