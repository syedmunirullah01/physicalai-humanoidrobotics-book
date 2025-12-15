# Chapter 3: Sensor Simulation - Package Files

This package contains sensor simulation examples for Gazebo: LiDAR, depth cameras, and IMUs.

## Package Contents

```
ch3_sensor_simulation/
├── config/                  # Sensor parameter YAML files
│   ├── lidar_params.yaml    # 2D LiDAR configuration (SICK TiM561 specs)
│   ├── camera_params.yaml   # RGB-D camera configuration (RealSense D435i specs)
│   └── imu_params.yaml      # IMU configuration (MPU-6050 specs)
├── urdf/                    # Robot URDFs with sensors
│   └── sensor_platform.urdf # Multi-sensor robot platform
├── launch/                  # Launch files for each sensor type
│   ├── lidar_2d.launch.py   # Spawn robot with LiDAR
│   ├── depth_camera.launch.py  # Spawn robot with depth camera
│   └── imu_sensor.launch.py    # Spawn robot with IMU
├── validation/              # Ground truth validation scripts
│   ├── ground_truth_distances.py  # Validate LiDAR against known distances
│   └── sensor_noise_analysis.py  # Analyze sensor noise characteristics
└── README.md                # This file
```

## Quick Start

### 1. Build Package

```bash
cd ~/ros2_ws
colcon build --packages-select ch3_sensor_simulation
source install/setup.bash
```

### 2. Launch Sensor Examples

**LiDAR Example**:
```bash
ros2 launch ch3_sensor_simulation lidar_2d.launch.py

# In another terminal, visualize in RViz2
rviz2
# Add → LaserScan → Topic: /scan
```

**Depth Camera Example**:
```bash
ros2 launch ch3_sensor_simulation depth_camera.launch.py

# Visualize RGB image
ros2 run rqt_image_view rqt_image_view /camera/rgb/image_raw

# Visualize depth map
ros2 run rqt_image_view rqt_image_view /camera/depth/image_raw

# Visualize point cloud in RViz2
rviz2
# Add → PointCloud2 → Topic: /camera/depth/points
```

**IMU Example**:
```bash
ros2 launch ch3_sensor_simulation imu_sensor.launch.py

# Monitor IMU data
ros2 topic echo /imu

# Visualize in RViz2
rviz2
# Add → Imu → Topic: /imu
```

## Sensor Configurations

### LiDAR (lidar_params.yaml)

Based on **SICK TiM561** specifications:
- **Range**: 0.05m - 10m
- **FOV**: 270° (±135°)
- **Resolution**: 0.25° (1081 samples)
- **Update Rate**: 15 Hz
- **Noise**: σ = 1cm (Gaussian)

### Depth Camera (camera_params.yaml)

Based on **Intel RealSense D435i** specifications:
- **Resolution**: 640x480 @ 30 FPS
- **Depth Range**: 0.3m - 10m
- **FOV**: 87° H x 58° V
- **Depth Accuracy**: < 2% at 2m (σ = 7mm)

### IMU (imu_params.yaml)

Based on **MPU-6050** specifications:
- **Gyroscope**: ±500°/s range, σ = 0.2°/s noise
- **Accelerometer**: ±2g range, σ = 0.17% of g noise
- **Update Rate**: 100 Hz

## Validation Scripts

### Ground Truth Distance Validation

Tests LiDAR accuracy against known obstacle distances:

```bash
# Place obstacles at 1m, 3m, 5m in Gazebo
# Run validation
ros2 run ch3_sensor_simulation ground_truth_distances.py

# Expected output:
# Obstacle 1: Measured 1.01m, Ground Truth 1.00m, Error: 1.0%
# Obstacle 2: Measured 2.98m, Ground Truth 3.00m, Error: 0.7%
# Obstacle 3: Measured 5.03m, Ground Truth 5.00m, Error: 0.6%
# ✅ All errors < 2% - PASS
```

### Sensor Noise Analysis

Analyzes sensor noise characteristics over time:

```bash
ros2 run ch3_sensor_simulation sensor_noise_analysis.py

# Collects 1000 samples, calculates statistics:
# Mean: 2.501m
# Std Dev: 0.011m (expected: 0.01m from config)
# Min: 2.475m
# Max: 2.528m
# ✅ Noise model matches configuration - PASS
```

## URDF Sensor Platform

The `sensor_platform.urdf` combines all three sensor types on a single robot:

```xml
<robot name="sensor_platform">
  <link name="base_link">...</link>

  <!-- LiDAR mounted on top -->
  <link name="lidar_link">...</link>
  <gazebo reference="lidar_link">
    <sensor name="lidar" type="ray">...</sensor>
  </gazebo>

  <!-- Depth camera mounted in front -->
  <link name="camera_link">...</link>
  <gazebo reference="camera_link">
    <sensor name="depth_camera" type="depth">...</sensor>
  </gazebo>

  <!-- IMU in center of base -->
  <link name="imu_link">...</link>
  <gazebo reference="imu_link">
    <sensor name="imu" type="imu">...</sensor>
  </gazebo>
</robot>
```

## Customizing Sensor Parameters

### Method 1: Edit YAML Files

```yaml
# config/lidar_params.yaml
lidar_sensor:
  ros__parameters:
    range_max: 30.0        # Increase max range to 30m
    noise_stddev: 0.02     # Increase noise to 2cm
```

Then reload configuration (restart launch file).

### Method 2: Override in Launch File

```python
lidar_params = {
    'range_max': 30.0,
    'noise_stddev': 0.02
}

Node(
    package='ch3_sensor_simulation',
    executable='lidar_node',
    parameters=[lidar_params]
)
```

## RViz2 Visualization Tips

### LiDAR Visualization

```bash
rviz2
# Add → LaserScan
# Topic: /scan
# Fixed Frame: base_link
# Size: 0.05 (make rays visible)
# Color: By intensity or By range
```

### Depth Camera Visualization

```bash
rviz2
# For RGB image:
# Add → Image → Topic: /camera/rgb/image_raw

# For point cloud:
# Add → PointCloud2 → Topic: /camera/depth/points
# Style: Points (small size: 0.01)
# Color: RGB8 (if available) or Intensity
```

### IMU Visualization

```bash
rviz2
# Add → Imu
# Topic: /imu
# Show Acceleration: true
# Acceleration Scale: 0.5
# Show Axes: true
```

## Troubleshooting

### LiDAR shows no data

- **Check**: Robot is spawned and Gazebo is running
- **Check**: `/scan` topic exists (`ros2 topic list`)
- **Check**: Sensor link is above ground (not intersecting floor)
- **Fix**: Add obstacles in Gazebo for LiDAR to detect

### Depth camera image is black

- **Check**: Camera is pointing at objects (within 0.3-10m range)
- **Check**: Gazebo world has lighting (`<light>` in SDF)
- **Fix**: Add `<visualize>true</visualize>` to camera sensor for debug rays

### IMU data is all zeros

- **Check**: Robot is moving (IMU measures changes, not static state)
- **Fix**: Apply forces to robot or use velocity commands to move it
- **Fix**: Verify `always_on` is true in sensor configuration

## Integration with Chapter 1 & 2

### Use Sensors with Gazebo Physics (Chapter 1)

```bash
# Launch wheeled robot from Chapter 1 with LiDAR
ros2 launch ch1_gazebo_physics wheeled_robot.launch.py

# In another terminal, attach LiDAR sensor
# (Requires modifying diff_drive_robot.urdf to include sensor plugins)
```

### Use Sensors with Unity Rendering (Chapter 2)

```bash
# Terminal 1: Gazebo with sensors
ros2 launch ch3_sensor_simulation lidar_2d.launch.py

# Terminal 2: Unity ROS bridge
ros2 launch unity_bridge unity_ros_bridge.launch.py

# Unity can subscribe to /scan and visualize LiDAR rays in 3D scene
```

## Additional Resources

- **Gazebo Sensor Plugins**: https://classic.gazebosim.org/tutorials?tut=ros_gzplugins#Sensor
- **ROS 2 sensor_msgs**: https://docs.ros2.org/latest/api/sensor_msgs/
- **RViz2 Displays**: https://github.com/ros2/rviz/blob/rolling/docs/user_guide.md

## Testing

Run all validation tests:

```bash
colcon test --packages-select ch3_sensor_simulation
colcon test-result --verbose
```

Expected: All tests PASS
