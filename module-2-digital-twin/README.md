# Module 2: The Digital Twin (Gazebo & Unity) - Code Examples

**Physical AI & Humanoid Robotics Textbook**
**Module**: Module 2 - The Digital Twin (Gazebo & Unity)
**Description**: Complete code examples for physics simulation, high-fidelity rendering, and sensor simulation

---

## Overview

This repository contains all code examples, URDF models, Unity scenes, and launch files for **Module 2: The Digital Twin** of the Physical AI & Humanoid Robotics textbook. Students learn to build digital twins of robotic systems using Gazebo Harmonic for physics simulation, Unity for photorealistic rendering, and simulated sensors for perception development.

---

## Repository Structure

```
module-2-digital-twin/
├── src/
│   ├── ch1_gazebo_physics/       # Chapter 1: Physics Simulation in Gazebo
│   │   ├── launch/               # ROS 2 launch files
│   │   ├── urdf/                 # Robot URDF models with physics
│   │   └── worlds/               # Gazebo world files
│   ├── ch2_unity_rendering/      # Chapter 2: Unity Rendering & HRI
│   │   ├── UnityProject/         # Unity 2022 LTS project
│   │   └── ros2_workspace/       # ROS 2 bridge package
│   └── ch3_sensor_simulation/    # Chapter 3: Sensor Simulation
│       ├── launch/               # Sensor launch files
│       ├── urdf/                 # Robots with sensors
│       ├── config/               # Sensor parameters (YAML)
│       └── validation/           # Sensor validation scripts
├── tests/                        # pytest and Unity tests
├── docker/                       # Docker environment
└── .github/workflows/            # CI/CD pipelines
```

---

## Quick Start

### Prerequisites

- **ROS 2 Humble** or **ROS 2 Jazzy**
- **Gazebo Harmonic** (Fortress or Garden)
- **Unity 2022 LTS** (for Chapter 2 only)
- **Python 3.10+**
- **Docker** (recommended for consistent environment)

### Option 1: Docker (Recommended)

```bash
# Build Docker image
cd module-2-digital-twin
docker build -f docker/Dockerfile.gazebo -t module2-gazebo:humble .

# Run container with GUI support
xhost +local:docker  # Allow Docker to access display (Linux only)
docker run -it --rm \
  --name module2-dev \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd):/workspace/src/module-2-digital-twin \
  module2-gazebo:humble
```

### Option 2: Native Installation (Ubuntu 22.04/24.04)

```bash
# Install ROS 2 Humble + Gazebo Harmonic (see textbook installation guide)

# Clone repository
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <repository-url> module-2-digital-twin

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install
source install/setup.bash
```

---

## Chapter 1: Physics Simulation in Gazebo

**Learning Goal**: Understand how Gazebo simulates gravity, friction, collisions, and joint constraints.

### Run Examples

```bash
# Example 1: Spawn rigid body in Gazebo
ros2 launch ch1_gazebo_physics spawn_rigid_body.launch.py

# Example 2: Differential drive wheeled robot
ros2 launch ch1_gazebo_physics wheeled_robot.launch.py

# Example 3: 3-DOF articulated arm
ros2 launch ch1_gazebo_physics articulated_arm.launch.py
```

**Expected Behavior**: Robots spawn, fall due to gravity, collide with ground plane, and respond to physics forces.

---

## Chapter 2: High-Fidelity Rendering in Unity

**Learning Goal**: Create photorealistic simulation environments for human-robot interaction testing.

### Setup Unity Project

1. **Install Unity 2022 LTS** from [Unity Hub](https://unity.com/download)
2. **Install ROS-TCP-Connector**:
   - Open Unity Package Manager
   - Add package from git URL: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
3. **Open Unity Project**:
   ```bash
   cd src/ch2_unity_rendering/UnityProject
   # Open in Unity Hub
   ```

### Run Unity + ROS 2 Integration

```bash
# Terminal 1: Start ROS-TCP-Endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

# Terminal 2: Open Unity and press Play
# Robot should respond to ROS 2 commands in real-time
```

**Expected Behavior**: Unity scene renders at 30+ FPS, robot animates based on ROS 2 joint commands.

---

## Chapter 3: Sensor Simulation

**Learning Goal**: Configure LiDAR, depth cameras, and IMUs for perception algorithm development.

### Run Sensor Examples

```bash
# Example 1: 2D LiDAR sensor
ros2 launch ch3_sensor_simulation lidar_2d.launch.py
ros2 topic echo /scan  # Verify LaserScan messages

# Example 2: Depth camera (RGB-D)
ros2 launch ch3_sensor_simulation depth_camera.launch.py
ros2 run rviz2 rviz2  # Visualize depth image

# Example 3: IMU (Inertial Measurement Unit)
ros2 launch ch3_sensor_simulation imu_sensor.launch.py
ros2 topic echo /imu  # Verify IMU messages
```

**Expected Behavior**: Sensors publish data to ROS 2 topics, RViz2 visualizes point clouds and images.

---

## Testing

### Run Unit Tests

```bash
# Test Chapter 1 physics examples
colcon test --packages-select ch1_gazebo_physics
colcon test-result --all --verbose

# Run pytest tests
pytest tests/ch1_physics/ -v
pytest tests/ch3_sensors/ -v
```

### Run Unity Tests

```bash
# Open Unity Test Runner
# Window → General → Test Runner
# Run all tests
```

---

## CI/CD

This repository uses GitHub Actions for continuous integration:

- **Gazebo CI**: Tests all ROS 2 packages and launch files
- **Unity CI**: Builds Unity project and runs tests
- **Lint**: Runs `ruff` on Python code

See `.github/workflows/` for pipeline configurations.

---

## Troubleshooting

<details>
<summary><strong>Issue</strong>: Gazebo crashes with "Segmentation fault"</summary>

**Cause**: Incompatible GPU drivers or missing libraries

**Solution**:
```bash
# Update GPU drivers
sudo ubuntu-drivers autoinstall

# Or use software rendering
LIBGL_ALWAYS_SOFTWARE=1 gz sim
```
</details>

<details>
<summary><strong>Issue</strong>: Unity can't connect to ROS 2</summary>

**Cause**: ROS-TCP-Endpoint not running or wrong IP

**Solution**:
```bash
# Start ROS-TCP-Endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

# In Unity ROSConnection GameObject, set:
# - ROS IP Address: localhost (or ROS 2 machine IP)
# - ROS Port: 10000
```
</details>

<details>
<summary><strong>Issue</strong>: Docker container can't display Gazebo GUI</summary>

**Cause**: X server not accessible

**Solution**:
```bash
# Run xhost before starting container
xhost +local:docker

# For WSL2 on Windows:
# Install VcXsrv and set DISPLAY variable
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
```
</details>

---

## Contributing

Found a bug or have an improvement? Contributions welcome!

1. Fork this repository
2. Create a feature branch: `git checkout -b fix/chapter1-typo`
3. Make changes and test: `colcon build && colcon test`
4. Ensure CI passes
5. Submit a pull request

---

## License

Apache 2.0 - See LICENSE file for details.

---

## Additional Resources

- **Textbook**: [Physical AI & Humanoid Robotics Documentation](https://physicalai-textbook.com)
- **ROS 2 Docs**: [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)
- **Gazebo Harmonic Docs**: [https://gazebosim.org/docs/harmonic](https://gazebosim.org/docs/harmonic)
- **Unity Robotics Hub**: [https://github.com/Unity-Technologies/Unity-Robotics-Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)

---

**Happy Simulating! 🤖**
