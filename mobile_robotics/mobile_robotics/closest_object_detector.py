import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import LaserScan 
import numpy as np  

class LaserScanSub(Node): 
    def __init__(self): 
        super().__init__('laser_scan_subscriber') 
        self.sub = self.create_subscription(LaserScan, "scan", self.lidar_cb, 10) 
        self.lidar = LaserScan() # Data from lidar will be stored here. 
        timer_period = 0.05 # 20 Hz 
        self.timer = self.create_timer(timer_period, self.timer_callback) 
        self.get_logger().info("Node initialized!!!") 

    def timer_callback(self): 
        if self.lidar.ranges: 
            closest_range, theta_closest = self.get_closest_object() 
            if np.isinf(closest_range): 
                print("WARNING:  there are no objects around") 
            else:
                print("closest range: ", closest_range) 
                print("theta closest: ", theta_closest) 
        else: 
            print("No lidar data recieved yet") 

    def get_closest_object(self): 
        closest_range = min(self.lidar.ranges) # Get the distance to the closest object 
        closest_index = self.lidar.ranges.index(closest_range)  
        theta_closest = self.lidar.angle_min + closest_index*self.lidar.angle_increment 
        # Crop the angle to (-pi, pi] 
        theta_closest = np.arctan2(np.sin(theta_closest), np.cos(theta_closest)) 
        return closest_range, theta_closest 

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