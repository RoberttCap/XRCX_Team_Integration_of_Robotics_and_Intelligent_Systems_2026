import rclpy 

from rclpy.node import Node 

from sensor_msgs.msg import LaserScan 

 

class LaserScanSub(Node): 

    def __init__(self): 
        super().__init__('laser_scan_subscriber') 
        self.sub = self.create_subscription(LaserScan, "scan", self.lidar_cb, 10) 
        self.lidar = LaserScan() # Data from lidar will be stored here. 
        self.received_scan = False
        timer_period = 1.0 
        self.timer = self.create_timer(timer_period, self.timer_callback) 
        self.get_logger().info("Node initialized!!!") 

     

    def timer_callback(self):        
        print("angle_min: ", self.lidar.angle_min)
        print("angle_max: ", self.lidar.angle_max)
        print("range_min: ", self.lidar.range_min)
        print("range_max: ", self.lidar.range_max)
        print("header.frame_id: ", self.lidar.header.frame_id)
    
   
        print("The first component inside the ranges[] array: ", self.lidar.ranges[0])
        print("The last component inside the intensities[] array: ", self.lidar.intensities[-1])
      
 

 

    def lidar_cb(self, lidar_msg): 
        ## This function receives the ROS LaserScan message 
        self.lidar =  lidar_msg  
        self.received_scan = True

 

def main(args=None): 
    rclpy.init(args=args) 
    m_p=LaserScanSub() 
    rclpy.spin(m_p) 
    m_p.destroy_node() 
    rclpy.shutdown() 

     

if __name__ == '__main__': 
    main() 
