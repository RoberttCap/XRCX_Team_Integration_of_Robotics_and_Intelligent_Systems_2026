# Puzzlebot Simulation - Verification Guide

## PARTE 1: SIMULATION (Simulated Differential Drive Kinematics)

### ✅ Implemented Components:
- **differential_drive.py**: Uses differential drive kinematic model to calculate pose (x, y, θ)
- Calculates wheel angular velocities (ω_r, ω_l) from cmd_vel commands
- Publishes simulated pose as `/pos_sim` (PoseStamped)
- Publishes wheel velocities as `/wheel_velocities` (Float64MultiArray)

### Verification Steps:

#### 1. Verify Kinematic Model with Constant Velocity
```bash
# Terminal 1: Launch simulation
ros2 launch puzzlebot_sim puzzlebot_launch.py

# Terminal 2: Send constant forward velocity for 10 seconds
for i in {1..10}; do
  ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}" --rate 1
done

# OR: Continuous at 10 Hz
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}" --rate 10 &
sleep 10
pkill -f "topic pub"
```

**Expected Result**: 
- In RViz: Robot moves forward in straight line
- Distance = v × t = 0.5 m/s × 10 s = 5.0 m
- Check `/pos_sim` topic shows position incrementing from 0 to 5.0 m

#### 2. Verify Different Velocities (Linear and Angular)
```bash
# Test 1: Pure linear motion (v=0.3 m/s, ω=0)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}}" --rate 10

# Test 2: Pure rotation (v=0, ω=0.5 rad/s)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}" --rate 10

# Test 3: Combined (v=0.3 m/s, ω=0.3 rad/s) - circular arc
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}, angular: {z: 0.3}}" --rate 10
```

#### 3. View Pose Changes in RViz
- Launch opens RViz automatically with fixed frame: `odom`
- RobotModel display shows Puzzlebot 3D model
- Watch as robot moves/rotates based on velocity commands
- Model uses STL meshes from Manchester Robotics

#### 4. Plot Position and Wheel Velocities with rqt_plot
```bash
# Terminal 2: Open rqt_plot
ros2 run rqt_plot rqt_plot /pos_sim/pose/position/x /pos_sim/pose/position/y \
  /wheel_velocities/data[0] /wheel_velocities/data[1]
```

**Graphs show**:
- **Line 1**: X position over time (increases with forward motion)
- **Line 2**: Y position over time (increases with left motion)
- **Line 3**: Right wheel angular velocity (rad/s)
- **Line 4**: Left wheel angular velocity (rad/s)

**Kinematics Equations**:
```
ω_r = (v + ω×L/2) / r
ω_l = (v - ω×L/2) / r

where:
  L = 0.19 m (wheel base)
  r = 0.05 m (wheel radius)
  v = linear velocity
  ω = angular velocity
```

---

## PARTE 2: LOCALISATION (Odometry Publishing)

### ✅ Implemented Components:
- **localization.py**: Subscribes to `/pos_sim` (simulated pose)
- Publishes odometry as `/odom` topic (nav_msgs/Odometry)
- Broadcasts TF transform: `odom` → `base_footprint`

### Verification Steps:

#### 1. Verify Localization Node Running
```bash
# Check if localization node is active
ros2 node list | grep localization

# Should output: /localization
```

#### 2. Verify Odometry Messages
```bash
# Monitor odometry messages in real-time
ros2 topic echo /odom --csv | head -20

# OR: Check message frequency
ros2 topic hz /odom
```

**Expected Output**:
- Position fields: `pose.pose.position.x/y/z`
- Orientation as quaternion: `pose.pose.orientation.x/y/z/w`
- Velocity fields: `twist.twist.linear.x` and `twist.twist.angular.z`
- Publishing at ~20 Hz

#### 3. Verify TF Tree
```bash
# View complete TF tree
ros2 run tf2_tools view_frames
evince frames.pdf  # View the generated PDF

# Check specific transform
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint base_link
```

#### 4. Plot Odometry Data with rqt_plot
```bash
# Plot odometry position and orientation
ros2 run rqt_plot rqt_plot \
  /odom/pose/pose/position/x \
  /odom/pose/pose/position/y \
  /odom/twist/twist/linear/x \
  /odom/twist/twist/angular/z
```

#### 5. Verify URDF Model in RViz
- RViz shows Puzzlebot with proper links:
  - `base_footprint` (reference frame)
  - `base_link` (main body with STL mesh)
  - `wheel_r_link` (right wheel with rotation)
  - `wheel_l_link` (left wheel with rotation)
  - `caster_link` (caster wheel)

---

## PARTE 3: CONTROL (Trajectory Tracking)

### ✅ Implemented Components:
- **controller.py**: Implements go-to-goal waypoint controller
- Waypoints hardcoded as: (1.0, 0.0) → (1.0, 1.0) → (0.0, 1.0) → (0.0, 0.0) → repeat
- Publishes velocity commands to `/cmd_vel`
- Subscribes to `/odom` for position feedback

### Verification Steps:

#### 1. Verify Controller Node Running
```bash
# Check controller is active
ros2 node list | grep controller

# Should output: /controller
```

#### 2. Verify Square Trajectory Generation
```bash
# Monitor cmd_vel commands being sent
ros2 topic echo /cmd_vel

# Expected: Alternating between forward motion and rotation
# to reach each corner of the square
```

#### 3. Watch Trajectory in RViz
1. Open RViz (automatically launched)
2. Set Fixed Frame to `odom`
3. Watch robot follow square path:
   - Start at (0, 0)
   - Move to (1, 0) - forward 1 m
   - Move to (1, 1) - turn left, forward 1 m
   - Move to (0, 1) - turn left, forward 1 m
   - Move to (0, 0) - turn left, forward 1 m
   - Repeat

#### 4. Plot Desired vs Actual Trajectory
```bash
# In a separate terminal, visualize position over time
ros2 run rqt_plot rqt_plot /pos_sim/pose/position/x /pos_sim/pose/position/y
```

**Expected Graph Pattern**:
- X axis: Oscillates 0 ↔ 1 (each edge takes ~10 seconds)
- Y axis: Oscillates 0 ↔ 1 (each edge takes ~10 seconds)
- Forms rough square shape over time

#### 5. Controller Parameters (Customizable)
```python
# In controller.py, adjust:
self.waypoints = [
    (1.0, 0.0),
    (1.0, 1.0),
    (0.0, 1.0),
    (0.0, 0.0)
]

self.kp_linear = 0.7      # Proportional gain for linear velocity
self.kp_angular = 2.0     # Proportional gain for angular velocity
self.max_linear = 0.5     # Max forward speed (m/s)
self.max_angular = 1.5    # Max rotation speed (rad/s)
self.position_tolerance = 0.05  # Distance to waypoint (m)
self.angle_tolerance = 0.15     # Angle to target (rad)
```

#### 6. Test Different Trajectory Shapes

**To create a triangle**:
```python
self.waypoints = [
    (0.0, 0.0),
    (1.0, 0.0),
    (0.5, 0.866),  # 60° angle for equilateral triangle
]
```

**To create a rectangle**:
```python
self.waypoints = [
    (0.0, 0.0),
    (1.5, 0.0),   # Longer edge
    (1.5, 0.8),   # Shorter edge
    (0.0, 0.8),
]
```

---

## Complete Workflow to Verify All 3 Parts

```bash
# Terminal 1: Start full simulation
ros2 launch puzzlebot_sim puzzlebot_launch.py

# Wait 3 seconds for all nodes to start, then:

# Terminal 2: Monitor rqt_plot for all data
ros2 run rqt_plot rqt_plot \
  /pos_sim/pose/position/x \
  /pos_sim/pose/position/y \
  /wheel_velocities/data[0] \
  /wheel_velocities/data[1] \
  /odom/pose/pose/position/x \
  /odom/pose/pose/position/y

# Terminal 3: Monitor rqt_graph to see node connections
ros2 run rqt_graph rqt_graph

# Terminal 4: Monitor odometry (optional)
ros2 topic echo /odom
```

### Expected Behavior:
1. **RViz**: Puzzlebot model traces a square path automatically
2. **rqt_plot**: Position graphs form square pattern
3. **rqt_plot**: Wheel velocities change as robot navigates corners
4. **rqt_graph**: Shows all nodes connected via topics
5. **Console**: Logs show robot reaching each waypoint

---

## Topic Summary

| Topic | Type | Publisher | Subscriber | Purpose |
|-------|------|-----------|-----------|---------|
| `/cmd_vel` | Twist | controller | differential_drive | Velocity commands |
| `/pos_sim` | PoseStamped | differential_drive | localization, joint_state_publisher | Simulated pose |
| `/wheel_velocities` | Float64MultiArray | differential_drive | joint_state_publisher | Wheel angular velocities |
| `/odom` | Odometry | localization | rviz, controller | Odometry messages |
| `/joint_states` | JointState | joint_state_publisher | robot_state_publisher | Joint angles for visualization |
| `/robot_description` | String | robot_state_publisher | rviz, joint_state_publisher | URDF model |

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Robot not moving in RViz | Check if `/cmd_vel` topic being published: `ros2 topic echo /cmd_vel` |
| rqt_plot shows no data | Ensure simulation is running, check topic names with `ros2 topic list` |
| Robot "teleporting" instead of moving smoothly | Use `--rate 10` with lower velocity (e.g., 0.5 m/s) |
| Model not visible in RViz | Set Fixed Frame to `odom`, enable RobotModel display |
| No trajectory shape visible | Controller needs ~2-3 minutes to complete square. Be patient. |
