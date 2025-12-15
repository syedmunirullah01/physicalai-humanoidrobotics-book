# Recorded Trajectories - ROS 2 Bags for VSLAM

**Purpose**: Pre-recorded sensor data for Chapter 2 VSLAM exercises

This directory contains ROS 2 bag files with recorded stereo camera data from Isaac Sim, allowing learners to test VSLAM algorithms without running the full simulation.

## Available Trajectories

### 1. warehouse_loop_100m.db3
**Description**: 100-meter loop trajectory in warehouse environment with loop closure opportunity

**Statistics**:
- **Duration**: 200 seconds
- **Path Length**: 100.3 meters
- **Recording FPS**: 30 Hz
- **Loop Closure**: Yes (at 95m)
- **Difficulty**: Easy (good lighting, rich features)

**Topics**:
```
/camera/left/image_raw          (sensor_msgs/Image)       - 6000 messages
/camera/left/camera_info        (sensor_msgs/CameraInfo)  - 6000 messages
/camera/right/image_raw         (sensor_msgs/Image)       - 6000 messages
/camera/right/camera_info       (sensor_msgs/CameraInfo)  - 6000 messages
/ground_truth/pose              (geometry_msgs/PoseStamped) - 6000 messages
/tf                             (tf2_msgs/TFMessage)      - 12000 messages
```

**Use Cases**:
- Chapter 2, Exercise 1: Visual odometry basics
- Chapter 2, Exercise 2: Map building and loop closure
- SC-003 validation: <2% drift over 100m

**Expected VSLAM Performance**:
- Frame Rate: 30-60 Hz (depending on GPU)
- Trajectory Error: 1.2-1.8% drift before loop closure
- Final Error: <0.5% after loop closure
- Tracking Success: >98%

---

### 2. warehouse_corridor_50m.db3
**Description**: Straight corridor trajectory with challenging lighting

**Statistics**:
- **Duration**: 100 seconds
- **Path Length**: 50.2 meters
- **Recording FPS**: 30 Hz
- **Loop Closure**: No
- **Difficulty**: Medium (variable lighting, some featureless sections)

**Use Cases**:
- Chapter 2, Exercise 3: Handling tracking loss
- Chapter 2, Exercise 7: VSLAM debugging

**Expected VSLAM Performance**:
- Frame Rate: 30-60 Hz
- Trajectory Error: 1.5-2.5% drift
- Tracking Success: 92-95%
- Known Challenges: Tracking loss at t=45s (featureless wall)

---

### 3. warehouse_complex_200m.db3
**Description**: Complex multi-room trajectory with dynamic obstacles

**Statistics**:
- **Duration**: 400 seconds
- **Path Length**: 203.7 meters
- **Recording FPS**: 30 Hz
- **Loop Closure**: Yes (multiple)
- **Difficulty**: Hard (dynamic objects, occlusions, varying speeds)

**Use Cases**:
- Chapter 2, Exercise 4: Advanced VSLAM scenarios
- SC-002 validation: >30 Hz real-time performance
- Full integration testing

**Expected VSLAM Performance**:
- Frame Rate: 30-45 Hz (demanding)
- Trajectory Error: 1.8-3.0% drift
- Tracking Success: 88-93%
- Known Challenges: Dynamic objects at t=120s, t=280s

---

## Recording These Bags Yourself

**Option 1: Record from Isaac Sim** (Recommended for learning)

```bash
# Terminal 1: Launch Isaac Sim with warehouse scene
cd chapter-2-isaac-ros/launch
ros2 launch vslam-sim.launch.py record:=true

# Terminal 2: Drive the robot along trajectory
# Use teleop or predefined waypoint navigation
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 3: Monitor recording
ros2 bag info rosbag2_warehouse_recording/rosbag2_warehouse_recording_0.db3
```

**Option 2: Download Pre-recorded** (For testing/validation)

```bash
# Download from project releases
wget https://github.com/YOUR_ORG/physicalai-humanoidrobotics-book/releases/download/v1.0/recorded-trajectories.tar.gz

# Extract
tar -xzf recorded-trajectories.tar.gz -C shared/datasets/recorded-trajectories/
```

**Option 3: Use Minimal Test Bag** (Quick setup)

```bash
# Create a minimal 10-second test bag for code validation
python3 scripts/create_test_rosbag.py \
    --output shared/datasets/recorded-trajectories/test_10s.db3 \
    --duration 10
```

## Playing Back Bags

### Basic Playback

```bash
# Play bag at recorded speed
ros2 bag play shared/datasets/recorded-trajectories/warehouse_loop_100m.db3

# Play at half speed for debugging
ros2 bag play shared/datasets/recorded-trajectories/warehouse_loop_100m.db3 --rate 0.5

# Play in loop for continuous testing
ros2 bag play shared/datasets/recorded-trajectories/warehouse_loop_100m.db3 --loop
```

### Playback with VSLAM

```bash
# Terminal 1: Play bag
ros2 bag play shared/datasets/recorded-trajectories/warehouse_loop_100m.db3

# Terminal 2: Launch cuVSLAM
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py

# Terminal 3: Visualize in RViz
ros2 run rviz2 rviz2 -d chapter-2-isaac-ros/config/vslam.rviz
```

## Bag File Format

All bags use ROS 2 SQLite3 format (`.db3`):

```bash
# Inspect bag metadata
ros2 bag info warehouse_loop_100m.db3

# Output example:
# Files:             warehouse_loop_100m.db3
# Bag size:          2.1 GB
# Storage id:        sqlite3
# Duration:          200.0s
# Start:             Dec 14 2025 10:15:32.123
# End:               Dec 14 2025 10:18:52.456
# Messages:          30000
# Topic information:
#   /camera/left/image_raw: 6000 messages (sensor_msgs/Image)
#   /camera/right/image_raw: 6000 messages (sensor_msgs/Image)
#   ...
```

## Ground Truth Data

Each bag includes `/ground_truth/pose` for validation:

```python
# Compare VSLAM estimate vs ground truth
from shared.utils.metrics import VSLAMMetrics

vslam_metrics = VSLAMMetrics()

# Subscribe to both topics
# /visual_slam/tracking/odometry (VSLAM estimate)
# /ground_truth/pose (ground truth)

# Calculate trajectory error
def calculate_error(vslam_pose, gt_pose):
    dx = vslam_pose.position.x - gt_pose.position.x
    dy = vslam_pose.position.y - gt_pose.position.y
    dz = vslam_pose.position.z - gt_pose.position.z
    error = math.sqrt(dx**2 + dy**2 + dz**2)
    return error

# Validate SC-003: <2% drift over 100m
path_length = 100.3  # meters
final_error = 1.2    # meters (example)
drift_percent = (final_error / path_length) * 100

assert drift_percent < 2.0, f"Drift {drift_percent:.2f}% exceeds 2% threshold"
```

## Camera Calibration

All bags use the same stereo camera calibration:

**Camera Specifications**:
- **Baseline**: 120mm
- **Resolution**: 848x480 (optimized for VSLAM)
- **FPS**: 30 Hz
- **FOV**: 90° horizontal

**Intrinsics** (both cameras identical):
```yaml
K: [426.67, 0.0, 424.0,
    0.0, 426.67, 240.0,
    0.0, 0.0, 1.0]

D: [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion (rectified)

R: [1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0]

P: [426.67, 0.0, 424.0, 0.0,      # Left camera
    0.0, 426.67, 240.0, 0.0,
    0.0, 0.0, 1.0, 0.0]

# Right camera P matrix includes baseline
P_right: [426.67, 0.0, 424.0, -51.2,  # -fx * baseline
          0.0, 426.67, 240.0, 0.0,
          0.0, 0.0, 1.0, 0.0]
```

## Storage Requirements

| Bag File | Size | Compression |
|----------|------|-------------|
| warehouse_loop_100m.db3 | ~2.1 GB | None |
| warehouse_corridor_50m.db3 | ~1.1 GB | None |
| warehouse_complex_200m.db3 | ~4.3 GB | None |
| **Total** | **~7.5 GB** | |

With LZ4 compression (recommended):
| Bag File | Compressed Size | Ratio |
|----------|-----------------|-------|
| warehouse_loop_100m.db3 | ~850 MB | 2.5:1 |
| warehouse_corridor_50m.db3 | ~450 MB | 2.4:1 |
| warehouse_complex_200m.db3 | ~1.7 GB | 2.5:1 |
| **Total** | **~3 GB** | |

## Tips for Efficient Playback

1. **Use appropriate playback rate**:
   - Full speed (1.0x) for real-time performance testing
   - Half speed (0.5x) for detailed observation
   - Double speed (2.0x) for quick testing

2. **Limit topics if needed**:
   ```bash
   # Play only left camera (mono VSLAM)
   ros2 bag play warehouse_loop_100m.db3 \
       --topics /camera/left/image_raw /camera/left/camera_info
   ```

3. **Start from specific timestamp**:
   ```bash
   # Skip first 30 seconds
   ros2 bag play warehouse_loop_100m.db3 --start-offset 30
   ```

4. **Publish clock for simulation**:
   ```bash
   # Use bag time instead of system time
   ros2 bag play warehouse_loop_100m.db3 --clock
   ```

## Validation Checklist

Before using these bags for Chapter 2 exercises:

- [ ] Verify bag file integrity: `ros2 bag info <bag_file>`
- [ ] Check topic availability: All 6 camera/pose topics present
- [ ] Confirm message counts: ~6000 images per 200s bag
- [ ] Test playback: Bag plays without errors
- [ ] Validate timestamps: Monotonically increasing, no gaps
- [ ] Check image quality: Images display correctly in RViz

## Troubleshooting

**Issue**: Bag playback stutters or drops frames
- **Solution**: Increase ROS 2 buffer size or reduce playback rate

**Issue**: VSLAM not receiving images
- **Solution**: Check topic remapping in VSLAM launch file

**Issue**: Ground truth poses not available
- **Solution**: Ensure `/ground_truth/pose` topic is included in playback

## License

Part of the Physical AI & Humanoid Robotics educational series.
For educational use only.

## References

- ROS 2 Bag Documentation: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data.html
- Isaac ROS Visual SLAM: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/
