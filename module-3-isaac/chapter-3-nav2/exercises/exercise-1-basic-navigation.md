# Exercise 1: Basic Navigation with Isaac ROS Nav2

## Objective
Learn to set up and execute basic navigation using Isaac ROS Nav2 with VSLAM integration.

## Prerequisites
- Chapter 2 VSLAM implementation completed
- Robot with stereo cameras and IMU
- ROS 2 Humble installed
- Isaac ROS packages installed

## Steps

### 1. Environment Setup
1. Launch Isaac Sim or your robot platform
2. Start the stereo camera and IMU nodes
3. Verify VSLAM is working by running:
   ```bash
   ros2 launch isaac_ros_vslam vslam-sim.launch.py
   ```

### 2. Launch Nav2
1. Open a new terminal and source ROS 2
2. Launch Nav2 with VSLAM integration:
   ```bash
   ros2 launch isaac_ros_nav2 nav2-sim.launch.py
   ```

### 3. Visualize in RViz
1. Launch RViz with the Nav2 configuration:
   ```bash
   rviz2 -d $(ros2 pkg prefix isaac_ros_nav2)/share/isaac_ros_nav2/config/nav2.rviz
   ```
2. Observe the costmaps, path planning, and robot position

### 4. Send Navigation Goal
1. In RViz, click the "2D Goal Pose" button
2. Click and drag to set the goal position and orientation
3. Watch the robot plan and execute the path

### 5. Monitor Navigation
1. Observe the global and local costmaps
2. Watch the path replanning as the robot moves
3. Note how the robot avoids obstacles

## Expected Results
- Robot successfully navigates to the goal
- VSLAM provides accurate localization
- Costmaps update in real-time
- Recovery behaviors activate when needed

## Troubleshooting
- If localization fails, check VSLAM odometry topic
- If navigation doesn't start, verify costmap is not fully inflated
- If robot gets stuck, check laser scan topic availability

## Next Steps
- Try Exercise 2: Footstep Planning
- Experiment with different recovery behaviors
- Test navigation with dynamic obstacles