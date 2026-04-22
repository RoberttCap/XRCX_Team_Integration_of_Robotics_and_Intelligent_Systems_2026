# Puzzlebot ROS 2 Simulation Package

This package implements a simple differential-drive simulation stack for a Puzzlebot in ROS 2 using `rclpy`.

The project is organized in three incremental parts:

1. `Part 1`: kinematic simulation of the robot driven by `/cmd_vel`
2. `Part 2`: dead-reckoning odometry and visualization support
3. `Part 3`: TF publishing, wheel joint states, closed-loop go-to-goal control, and interactive goal input

The package does not use Gazebo or a physics engine. The robot motion is generated with a kinematic model and Euler integration.

## What This Project Does

At a high level, the package does the following:

- Reads velocity commands from `/cmd_vel`
- Simulates the Puzzlebot pose `(x, y, theta)`
- Converts body velocities into wheel angular speeds
- Reconstructs odometry from wheel speeds
- Publishes TF and wheel joint states for RViz visualization
- Accepts pose goals through `/goal_pose`
- Drives the robot to the requested pose with a nonlinear proportional controller
- Lets you type absolute goals or figures like `square` and `pentagon` from the terminal

## Package Structure

```text
puzzlebot_sim/
├── launch/
│   ├── part1.launch.py
│   ├── part2.launch.py
│   └── part3.launch.py
├── meshes/
├── puzzlebot_sim/
│   ├── control.py
│   ├── goal_input.py
│   ├── joint_states.py
│   ├── localisation.py
│   └── puzzlebot_sim.py
├── rviz/
│   └── puzzlebot_rviz.rviz
├── urdf/
│   └── puzzlebot.urdf
├── package.xml
└── setup.py
```

## Main Nodes

### 1. `puzzlebot_sim`

File: `puzzlebot_sim/puzzlebot_sim.py`

Purpose:

- Simulates the robot as a differential-drive platform
- Subscribes to `/cmd_vel`
- Publishes wheel speeds and simulated pose signals

Subscriptions:

- `/cmd_vel` - `geometry_msgs/msg/Twist`

Publications:

- `/wr` - right wheel angular speed, `std_msgs/msg/Float32`
- `/wl` - left wheel angular speed, `std_msgs/msg/Float32`
- `/sim_x` - simulated x position
- `/sim_y` - simulated y position
- `/sim_theta` - simulated heading
- `/pose_sim` - `geometry_msgs/msg/PoseStamped`

Key model:

- Wheel radius: `0.05 m`
- Wheel base: `0.19 m`
- Integration rate: `50 Hz`

### 2. `localisation`

File: `puzzlebot_sim/localisation.py`

Purpose:

- Reconstructs the robot pose from wheel speeds
- Publishes `Odometry`

Subscriptions:

- `/wr`
- `/wl`

Publications:

- `/odom` - `nav_msgs/msg/Odometry`

Important detail:

- This node performs dead reckoning, so the estimated pose depends on the wheel-speed integration.

### 3. `joint_states`

File: `puzzlebot_sim/joint_states.py`

Purpose:

- Publishes the TF tree needed by RViz
- Publishes wheel joint states for the URDF model

Subscriptions:

- `/odom`
- `/wr`
- `/wl`

Publications:

- `/joint_states` - `sensor_msgs/msg/JointState`
- dynamic TF: `odom -> base_footprint`
- static TF: `map -> odom`

What it makes possible:

- RViz can animate the robot model and wheel motion using the URDF plus TF/joint states.

### 4. `control`

File: `puzzlebot_sim/control.py`

Purpose:

- Moves the robot from the current pose to a goal pose
- Uses a nonlinear proportional controller for a differential-drive robot

Subscriptions:

- `/odom` - robot estimated pose
- `/goal_pose` - `geometry_msgs/msg/Pose2D`

Publications:

- `/cmd_vel`

Controller behavior:

- Position error:

```text
dx = x_goal - x
dy = y_goal - y
distance_error = sqrt(dx^2 + dy^2)
desired_heading = atan2(dy, dx)
heading_error = atan2(sin(desired_heading - theta), cos(desired_heading - theta))
```

- Linear control:

```text
v = kv * distance_error * cos(heading_error)
```

- Angular control:

```text
w = kw * heading_error
```

- Extra improvements already implemented:

```text
if abs(heading_error) > 0.3:
    v = 0.0

if distance_error < 0.2:
    v = 0.5 * v
```

- Final pose alignment:
  - once the robot reaches the target position, it rotates in place until the final orientation is reached

- Limits:
  - `v_max = 0.25 m/s`
  - `w_max = 1.5 rad/s`

- Stop conditions:
  - position tolerance: `0.05 m`
  - orientation tolerance: `2 deg`

### 5. `goal_input`

File: `puzzlebot_sim/goal_input.py`

Purpose:

- Lets you type goals from the terminal
- Publishes absolute goals to `/goal_pose`
- Can generate figure trajectories waypoint-by-waypoint

Subscriptions:

- `/odom`

Publications:

- `/goal_pose`

Supported input modes:

- Absolute pose:

```text
x y theta_deg
```

Example:

```text
1.0 1.0 90
```

- Figures:

```text
square
square 0.5
pentagon
pentagon 1.2
```

Behavior of figure mode:

- The figure starts from the current robot pose
- The node generates absolute waypoints
- It waits until the current waypoint is reached
- Then it publishes the next waypoint automatically

## Main Topics

| Topic | Type | Produced by | Used by |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `control` or manual test | `puzzlebot_sim` |
| `/wr` | `std_msgs/msg/Float32` | `puzzlebot_sim` | `localisation`, `joint_states` |
| `/wl` | `std_msgs/msg/Float32` | `puzzlebot_sim` | `localisation`, `joint_states` |
| `/odom` | `nav_msgs/msg/Odometry` | `localisation` | `control`, `joint_states`, `goal_input` |
| `/goal_pose` | `geometry_msgs/msg/Pose2D` | `goal_input` or manual publish | `control` |
| `/joint_states` | `sensor_msgs/msg/JointState` | `joint_states` | `robot_state_publisher` / RViz |
| `/pose_sim` | `geometry_msgs/msg/PoseStamped` | `puzzlebot_sim` | debugging |

## TF Frames

The current TF chain expected by the project is:

```text
map -> odom -> base_footprint -> base_link -> wheels/caster
```

Where:

- `map -> odom` is static
- `odom -> base_footprint` is dynamic and comes from the odometry estimate
- `base_footprint -> base_link` and wheel joints come from the URDF

## Requirements

At minimum, you need a ROS 2 installation with:

- `rclpy`
- `geometry_msgs`
- `nav_msgs`
- `sensor_msgs`
- `std_msgs`
- `tf2_ros`
- `robot_state_publisher`
- `launch`
- `launch_ros`

For the visualization launches you also need:

- `rviz2`
- `rqt_graph`
- `rqt_plot`
- `rqt_tf_tree`

Optional but useful:

- `teleop_twist_keyboard`

## Build Instructions

From the workspace root:

```bash
cd ~/ros2_ws
colcon build --packages-select puzzlebot_sim
source install/setup.bash
```

If you open a new terminal, source the workspace again:

```bash
cd ~/ros2_ws
source install/setup.bash
```

## How To Run Each Launch

### `part1.launch.py`

Run:

```bash
ros2 launch puzzlebot_sim part1.launch.py
```

What it launches:

- `puzzlebot_sim`
- `rqt_graph`
- `rqt_plot`

What this part is for:

- Testing the basic kinematic simulator
- Sending velocity commands manually
- Checking that the wheel-speed equations and pose integration behave correctly

What you should expect:

- The robot will not move until something publishes to `/cmd_vel`
- `rqt_graph` should show `/cmd_vel -> puzzlebot_sim -> /wr, /wl, /sim_x, /sim_y, /sim_theta`
- `rqt_plot` can be used to visualize signals such as:
  - `/sim_x/data`
  - `/sim_y/data`
  - `/sim_theta/data`
  - `/wr/data`
  - `/wl/data`

How to test it:

In another terminal:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

Expected result:

- `/sim_x` should increase
- `/sim_y` should stay approximately constant
- `/sim_theta` should stay approximately constant
- `/wr` and `/wl` should be similar

Angular test:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

Expected result:

- `/sim_theta` should change
- `/wr` and `/wl` should have opposite signs

### `part2.launch.py`

Run:

```bash
ros2 launch puzzlebot_sim part2.launch.py
```

Intended purpose:

- Run simulator plus dead reckoning
- Visualize the robot in RViz
- Inspect graphs and plots

What the launch file currently tries to launch:

- `robot_state_publisher`
- `puzzlebot_sim`
- `localisation`
- `puzzlebot_transforms`
- `rviz2`
- `rqt_graph`
- `rqt_plot`

Important note about the current repository state:

- `part2.launch.py` currently references `puzzlebot_transforms`
- `setup.py` also exposes `puzzlebot_transforms = puzzlebot_sim.transforms:main`
- but `puzzlebot_sim/transforms.py` is not present in the source tree

This means:

- `part2.launch.py` is conceptually the "Part 2 + visualization" launch
- but in the current state of the repo it is expected to fail unless that missing node is restored or the launch is updated

What should be expected conceptually if this part were complete:

- `/odom` published by `localisation`
- RViz showing the robot model
- TF tree connecting the odometry frame and the robot body

Recommended practical alternative right now:

- Use `part3.launch.py` for the full working stack

### `part3.launch.py`

Run:

```bash
ros2 launch puzzlebot_sim part3.launch.py
```

What it launches:

- `robot_state_publisher`
- `puzzlebot_sim`
- `localisation`
- `joint_states`
- `control`
- `rviz2`
- `rqt_tf_tree`

What this part is for:

- Full closed-loop operation
- Odometry estimation
- TF and joint-state publishing
- RViz visualization
- Go-to-goal control

What you should expect immediately after launch:

- RViz opens with the Puzzlebot model
- `rqt_tf_tree` opens
- The robot stays still at startup
- The `control` node prints that it is waiting for a goal on `/goal_pose`

What you should expect after sending a goal:

- The controller publishes `/cmd_vel`
- The simulator moves the robot
- `localisation` updates `/odom`
- `joint_states` updates the wheel rotation and TF
- RViz shows the robot moving
- Once the position is reached, the robot aligns the final orientation
- When both position and orientation are reached, the controller stops the robot

## How To Send Goals

### Option 1. Use `goal_input`

Run in another terminal:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run puzzlebot_sim goal_input
```

Then type an absolute goal:

```text
1.0 1.0 90
```

Expected behavior:

- `/goal_pose` receives the target pose
- The controller logs the new goal
- The robot moves to `(1.0, 1.0)` and aligns near `90 deg`

### Option 2. Publish directly from the terminal

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/Pose2D "{x: 1.0, y: 1.0, theta: 1.57}"
```

Expected behavior:

- Same as above, but without the interactive helper node

## How To Draw Figures

After running:

```bash
ros2 run puzzlebot_sim goal_input
```

Type one of these commands:

```text
square
square 0.5
pentagon
pentagon 1.0
```

What happens:

- The node reads the robot's current pose from `/odom`
- It creates a list of absolute waypoints
- It publishes the first waypoint
- Once the controller reaches that waypoint, the next waypoint is sent automatically
- At the end, the terminal prints `Figure completed.`

Expected visual behavior:

- `square` should trace a four-sided path
- `pentagon` should trace a five-sided path
- The robot should pause only as needed for heading correction and waypoint transitions

## Manual Checks and Debugging Commands

Check active nodes:

```bash
ros2 node list
```

Check active topics:

```bash
ros2 topic list
```

Check `/cmd_vel` frequency:

```bash
ros2 topic hz /cmd_vel
```

Expected:

- near `50 Hz` while the controller is actively moving

Check odometry:

```bash
ros2 topic echo /odom
```

Expected:

- `pose.pose.position.x` and `pose.pose.position.y` change as the robot moves
- `twist.twist.linear.x` and `twist.twist.angular.z` reflect estimated motion

Check the goal topic:

```bash
ros2 topic echo /goal_pose
```

Check controller output:

```bash
ros2 topic echo /cmd_vel
```

Expected:

- non-zero velocities during motion
- zero velocities after reaching the goal

Inspect TF:

```bash
ros2 run tf2_tools view_frames
```

Or use `rqt_tf_tree` from `part3.launch.py`.

## What To Expect In RViz

When using the full stack:

- The robot model should appear using the URDF and mesh files
- Wheel joints should animate while the robot moves
- The robot body should move in the `odom` frame
- The pose should match the dead-reckoning estimate, not a physics-based ground truth

If RViz opens but the robot does not move:

- check `/cmd_vel`
- check `/odom`
- check `/joint_states`
- check the TF tree

## Known Limitations

1. `part2.launch.py` references a missing node:
   - `puzzlebot_transforms`
   - the corresponding source file is not currently in the repository

2. The simulator is purely kinematic:
   - no wheel slip
   - no actuator dynamics
   - no collisions
   - no physics engine

3. Odometry comes from integrated wheel speeds:
   - it is a dead-reckoning estimate
   - it does not include sensor fusion

4. `goal_input` is not included inside `part3.launch.py`:
   - you must run it in a separate terminal if you want interactive goals

## Typical Workflow

### Basic simulator test

```bash
cd ~/ros2_ws
colcon build --packages-select puzzlebot_sim
source install/setup.bash
ros2 launch puzzlebot_sim part1.launch.py
```

Then publish `/cmd_vel` manually.

### Full closed-loop test

Terminal 1:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch puzzlebot_sim part3.launch.py
```

Terminal 2:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run puzzlebot_sim goal_input
```

Then type:

```text
1.0 1.0 90
```

Or:

```text
square 0.5
```

## Summary

This package already contains a complete educational pipeline for:

- differential-drive kinematic simulation
- wheel-speed generation
- dead-reckoning odometry
- TF and joint-state publishing
- RViz robot visualization
- nonlinear go-to-goal control
- terminal-based pose goals
- figure generation with square and pentagon trajectories

For day-to-day use, the most useful entry point in the current repository state is:

```bash
ros2 launch puzzlebot_sim part3.launch.py
```

followed by:

```bash
ros2 run puzzlebot_sim goal_input
```
