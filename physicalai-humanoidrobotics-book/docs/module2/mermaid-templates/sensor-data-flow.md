# Sensor Data Flow Template (Sequence Diagram)

## Template Code

```mermaid
sequenceDiagram
    participant Gazebo as Gazebo Simulator
    participant Plugin as Gazebo Plugin<br/>(e.g., libgazebo_ros_camera)
    participant ROS as ROS 2 Topic<br/>(/sensor_topic)
    participant Node as ROS 2 Node<br/>(perception_node)
    participant RViz as RViz2 Visualizer

    Gazebo->>Plugin: Generate sensor data<br/>(50Hz)
    Plugin->>Plugin: Apply noise model<br/>(Gaussian, outliers)
    Plugin->>ROS: Publish message<br/>(sensor_msgs/LaserScan)
    ROS->>Node: Subscribe to topic
    Node->>Node: Process data<br/>(filtering, transforms)
    Node->>ROS: Publish processed<br/>(sensor_msgs/PointCloud2)
    ROS->>RViz: Display visualization

    Note over Gazebo,RViz: Pipeline: Physics → Sensor → ROS 2 → Processing → Visualization
```

## Customization Guide

1. **Participants**: Rename based on your pipeline (e.g., `Unity`, `custom_node`)
2. **Messages**: Update with actual message types (e.g., `sensor_msgs/Image`, `geometry_msgs/Twist`)
3. **Frequencies**: Add timing annotations (e.g., `(30Hz)`, `(10Hz)`)
4. **Processing Steps**: Expand `Node` processing with specific algorithms

## Example: LiDAR Perception Pipeline

```mermaid
sequenceDiagram
    participant Gazebo as Gazebo Harmonic
    participant LiDARPlugin as libgazebo_ros_ray_sensor
    participant ScanTopic as /scan<br/>(sensor_msgs/LaserScan)
    participant PerceptionNode as /obstacle_detector<br/>(Python Node)
    participant CmdVelTopic as /cmd_vel<br/>(geometry_msgs/Twist)
    participant RobotController as /diff_drive_controller
    participant RViz as RViz2

    Gazebo->>LiDARPlugin: Ray casting (360°)<br/>@10Hz
    LiDARPlugin->>LiDARPlugin: Add Gaussian noise<br/>(σ=0.01m)
    LiDARPlugin->>ScanTopic: Publish LaserScan<br/>(720 points)

    ScanTopic->>PerceptionNode: Subscribe
    PerceptionNode->>PerceptionNode: Detect obstacles<br/>(threshold: 0.5m)
    PerceptionNode->>CmdVelTopic: Publish velocity command<br/>(avoid obstacle)

    CmdVelTopic->>RobotController: Subscribe
    RobotController->>Gazebo: Apply wheel torques

    ScanTopic->>RViz: Visualize scan
    CmdVelTopic->>RViz: Visualize velocity

    Note over Gazebo,RViz: Closed-loop: Sense → Process → Act → Simulate
```

## Example: Depth Camera to Point Cloud

```mermaid
sequenceDiagram
    participant Gazebo as Gazebo
    participant DepthPlugin as libgazebo_ros_camera<br/>(depth mode)
    participant ImageTopic as /camera/depth/image_raw<br/>(sensor_msgs/Image)
    participant InfoTopic as /camera/depth/camera_info<br/>(sensor_msgs/CameraInfo)
    participant PointCloudNode as /depth_to_pointcloud
    participant PCLTopic as /camera/depth/points<br/>(sensor_msgs/PointCloud2)
    participant RViz as RViz2

    Gazebo->>DepthPlugin: Render depth buffer<br/>@30Hz
    DepthPlugin->>ImageTopic: Publish depth image<br/>(640x480, 16-bit)
    DepthPlugin->>InfoTopic: Publish camera intrinsics<br/>(K matrix)

    ImageTopic->>PointCloudNode: Subscribe to depth
    InfoTopic->>PointCloudNode: Subscribe to info
    PointCloudNode->>PointCloudNode: Reproject to 3D<br/>(using K matrix)
    PointCloudNode->>PCLTopic: Publish PointCloud2<br/>(~300k points)

    PCLTopic->>RViz: Display point cloud

    Note over Gazebo,RViz: Depth → 3D reconstruction pipeline
```

## Text Alternative Template

```markdown
<details>
<summary>Text alternative for Sensor Data Flow</summary>

This sequence diagram shows the data flow for [sensor type] in the simulation:

1. **Gazebo Simulator** generates sensor data at [frequency]
2. **Gazebo Plugin** ([plugin name]) processes raw data and applies noise
3. **ROS 2 Topic** ([topic name]) publishes [message type]
4. **ROS 2 Node** ([node name]) subscribes and processes data
5. **RViz2** visualizes the output for debugging

The pipeline demonstrates how simulated sensor data flows from physics simulation to ROS 2 nodes for perception algorithms.
</details>
```

## Common Patterns

### Sensor Simulation Only
```
Gazebo → Plugin → ROS Topic → RViz
```

### Closed-Loop Control
```
Gazebo → Sensor Plugin → ROS Topic → Perception Node → Control Node → Actuator Plugin → Gazebo
```

### Multi-Sensor Fusion
```
Gazebo → LiDAR Plugin → /scan
Gazebo → Camera Plugin → /image
Both → Fusion Node → /fused_data
```

## Timing Annotations

Add timing information to show latency:

```mermaid
sequenceDiagram
    participant A as Gazebo
    participant B as Plugin
    participant C as ROS Topic

    A->>B: Generate (t=0ms)
    Note over B: Processing<br/>(5ms)
    B->>C: Publish (t=5ms)
    Note over C: Network latency<br/>(1ms)
```
