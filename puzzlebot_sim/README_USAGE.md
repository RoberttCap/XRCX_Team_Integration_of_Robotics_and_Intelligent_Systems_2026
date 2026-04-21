# Puzzlebot Simulation - Usage Guide

## Overview
The Puzzlebot simulation consists of three main parts:
1. **SIMULATION**: Differential drive kinematics and wheel velocities
2. **LOCALIZATION**: Odometry from simulated pose
3. **CONTROL**: Waypoint tracking (square trajectory)

## Starting the Simulation

### Terminal 1 - Launch Main Simulation
```bash
cd ~/ros2_ws
ros2 launch puzzlebot_sim puzzlebot_launch.py
```

This launches:
- `robot_state_publisher` - publishes robot model transforms
- `joint_state_publisher` - publishes joint angles for wheel rotation
- `differential_drive` - simulates robot kinematics
- `localization` - publishes odometry
- `controller` - follows waypoints in a square
- `rviz2` - visualizes the robot

## Monitoring the Simulation

### Terminal 2 - Visualization Tools
```bash
ros2 launch puzzlebot_sim visualization_launch.py
```

This opens:
- **rqt_plot**: Shows real-time graphs of:
  - Position: `/pos_sim/pose/position/x`, `/pos_sim/pose/position/y`
  - Wheel velocities: `/wheel_velocities/data[0]` (right), `/wheel_velocities/data[1]` (left)
  
- **rqt_graph**: Shows the node graph and topic connections

### Optional: Manual Commands

Send velocity commands to test:
```bash
# Smooth continuous movement (10 Hz)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}" --rate 10

# Single command
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}, angular: {z: 0.2}}" --once
```

## Understanding the Data

### rqt_plot Topics:
- **x, y position**: Shows robot's 2D trajectory
- **wheel_velocities**: Shows angular velocities (rad/s) of left and right wheels
  - Should match kinematics: `omega_r = (v + omega*L/2) / r`
  - Should match kinematics: `omega_l = (v - omega*L/2) / r`

### Square Trajectory:
The controller automatically follows waypoints:
- (1.0, 0.0) → (1.0, 1.0) → (0.0, 1.0) → (0.0, 0.0) → repeat

You should see the actual path in rqt_plot matching the desired square pattern.

## Topics Available:
- `/cmd_vel` - input velocity commands
- `/pos_sim` - simulated pose (PoseStamped)
- `/wheel_velocities` - wheel angular velocities (Float64MultiArray)
- `/odom` - odometry message
- `/joint_states` - joint angles for visualization

## Nodes:
- `differential_drive` - kinematic simulation
- `localization` - odometry publisher
- `controller` - waypoint controller
- `joint_state_publisher` - wheel joint states for visualization
- `robot_state_publisher` - TF transforms
