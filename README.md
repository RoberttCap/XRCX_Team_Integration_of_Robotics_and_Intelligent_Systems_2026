# XRCX_Team_Integration_of_Robotics_and_Intelligent_Systems_2026

Repositorio para evidenciar los minichallenges a lo largo del semestre

## Integrantes

| Nombre | GitHub |
| --- | --- |
| Sofía Blanco Prigmore | `AifosWhite` |
| Josué Aldemar Garduño Gómez | `aldemar3002` |
| Karina Fernanda Maldonado Murillo | `thephoeniix` |
| Roberto Carlos Pedraza Miranda | `RoberttCap` |


# How to stop your robot before quitting 
''
  import signal  
  import sys 


 

    def shutdown_function(self, signum, frame): 
        # Handle shutdown gracefully 
        # This function will be called when Ctrl+C is pressed 
        # It will stop the robot and shutdown the node 
        self.get_logger().info("Shutting down. Stopping robot...") 
        stop_twist = Twist()  # All zeros to stop the robot 
        self.pub_cmd_vel.publish(stop_twist) # publish it to stop the robot before shutting down 
        rclpy.shutdown() # Shutdown the node 
        sys.exit(0) # Exit the program 
''
