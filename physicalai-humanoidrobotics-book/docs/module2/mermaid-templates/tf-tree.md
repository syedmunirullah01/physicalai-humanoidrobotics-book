# TF Tree Template

## Template Code

```mermaid
graph TD
    World["world<br/>(fixed frame)"]
    BaseLink["base_link<br/>(robot base)"]
    Sensor1["sensor_frame_1<br/>(sensor type)"]
    Sensor2["sensor_frame_2<br/>(sensor type)"]

    World --> BaseLink
    BaseLink --> Sensor1
    BaseLink --> Sensor2

    style World fill:#D0021B,stroke:#8B0000,color:#fff
    style BaseLink fill:#4A90E2,stroke:#2E5C8A,color:#fff
    style Sensor1 fill:#50E3C2,stroke:#2E8B7A,color:#000
    style Sensor2 fill:#50E3C2,stroke:#2E8B7A,color:#000
```

## Customization Guide

1. **Root Frame**: Typically `world` or `map` (fixed reference)
2. **Base Frame**: Robot's main body frame (`base_link`, `base_footprint`)
3. **Sensor Frames**: Child frames for sensors (e.g., `camera_link`, `lidar_link`)
4. **Relationships**: Parent → Child (arrow direction shows TF transform)

## Example: Humanoid Robot TF Tree

```mermaid
graph TD
    World["world"]
    Odom["odom"]
    BaseFootprint["base_footprint"]
    BaseLink["base_link"]

    Torso["torso_link"]
    Head["head_link"]

    CameraRGB["camera_rgb_frame<br/>(RGB Camera)"]
    CameraDepth["camera_depth_frame<br/>(Depth Camera)"]
    LiDAR["lidar_link<br/>(LiDAR)"]
    IMU["imu_link<br/>(IMU)"]

    World --> Odom
    Odom --> BaseFootprint
    BaseFootprint --> BaseLink
    BaseLink --> Torso
    Torso --> Head

    Head --> CameraRGB
    Head --> CameraDepth
    Torso --> LiDAR
    BaseLink --> IMU

    style World fill:#D0021B,stroke:#8B0000,color:#fff
    style Odom fill:#D0021B,stroke:#8B0000,color:#fff
    style BaseFootprint fill:#4A90E2,stroke:#2E5C8A,color:#fff
    style BaseLink fill:#4A90E2,stroke:#2E5C8A,color:#fff
    style Torso fill:#4A90E2,stroke:#2E5C8A,color:#fff
    style Head fill:#4A90E2,stroke:#2E5C8A,color:#fff
    style CameraRGB fill:#50E3C2,stroke:#2E8B7A,color:#000
    style CameraDepth fill:#50E3C2,stroke:#2E8B7A,color:#000
    style LiDAR fill:#50E3C2,stroke:#2E8B7A,color:#000
    style IMU fill:#50E3C2,stroke:#2E8B7A,color:#000
```

## Text Alternative Template

```markdown
<details>
<summary>Text alternative for TF Tree</summary>

This diagram shows the coordinate frame hierarchy for [robot name]:
- **world** (fixed global frame)
  - **odom** (odometry frame, tracks robot motion)
    - **base_footprint** (ground projection of robot)
      - **base_link** (robot's main body)
        - **sensor_frame_1** (sensor type, position relative to base)
        - **sensor_frame_2** (sensor type, position relative to base)

All sensor measurements are transformed through this hierarchy to the world frame for localization and mapping.
</details>
```

## Common Patterns

### Mobile Robot (Wheeled)
```
world → odom → base_footprint → base_link → [sensors]
```

### Humanoid Robot (Multi-Joint)
```
world → odom → base_footprint → base_link → torso → head → [sensors]
                                          → left_arm → [gripper]
                                          → right_arm → [gripper]
```

### Fixed Sensor Rig
```
world → sensor_mount → camera_link → camera_rgb_frame
                                    → camera_depth_frame
```

## Validation Commands

```bash
# View TF tree in terminal
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo world base_link

# Visualize in RViz2
ros2 run rviz2 rviz2
# Add TF display plugin
```
