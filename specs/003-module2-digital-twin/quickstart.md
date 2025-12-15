# Quickstart Guide: Module 2 - The Digital Twin

**Feature**: Module 2 - The Digital Twin (Gazebo & Unity)
**Date**: 2025-12-12
**Audience**: Students, content creators, and contributors

---

## Overview

This guide helps you get started with Module 2's simulation environment. You'll set up Gazebo Harmonic, Unity 2022 LTS, and run your first digital twin example in under 30 minutes.

---

## Prerequisites

### Knowledge Requirements

✅ **Required** (from Module 1):
- ROS 2 basics: nodes, topics, launch files
- URDF fundamentals: links, joints, visual/collision meshes
- Command-line proficiency (bash, `colcon`, `ros2 cli`)

❌ **Not Required** (we'll teach you):
- Gazebo experience (we'll start from scratch)
- Unity development (step-by-step tutorials provided)
- Physics simulation theory (covered in Chapter 1)

### Hardware Requirements

**Minimum Specs (Tier A - Simulation Only)**:
- **CPU**: Quad-core (Intel i5 / AMD Ryzen 5 or better)
- **RAM**: 8GB (12GB+ recommended for Unity)
- **GPU**: Integrated graphics (e.g., Intel UHD 620) - basic Gazebo rendering
- **Storage**: 20GB free (Gazebo + Unity + code examples)
- **OS**: Ubuntu 22.04 or 24.04 (native or WSL2)

**Recommended Specs (Better Performance)**:
- **CPU**: 6+ cores
- **RAM**: 16GB+
- **GPU**: Dedicated (NVIDIA GTX 1650 or AMD RX 570) - smoother Unity rendering
- **Storage**: SSD for faster Gazebo loading

**Optional (Tier B/C - Physical Hardware)**:
- NVIDIA Jetson Orin Nano/NX ($500-700)
- Intel RealSense D435i depth camera ($200)
- Unitree Go2/G1 robot ($1,600+) - for Tier C only

---

## Software Installation

### Option 1: Docker (Recommended for Beginners)

**Pros**: Pre-configured environment, no dependency conflicts
**Cons**: Slightly slower performance than native installation

#### Step 1: Install Docker

**Ubuntu 22.04/24.04**:
```bash
# Install Docker Engine
sudo apt update
sudo apt install -y docker.io docker-compose
sudo systemctl enable --now docker

# Add your user to docker group (avoids sudo for docker commands)
sudo usermod -aG docker $USER
newgrp docker  # Activate group membership
```

**Windows 10/11** (WSL2):
1. Install [Docker Desktop for Windows](https://www.docker.com/products/docker-desktop/)
2. Enable WSL2 integration in Docker Desktop settings
3. Open Ubuntu WSL terminal and verify: `docker --version`

#### Step 2: Pull Module 2 Docker Image

```bash
# Pull pre-built image with Gazebo + ROS 2 Humble
docker pull physicalai/module2-gazebo:humble

# Run container with GUI support (for Gazebo/RViz2)
xhost +local:docker  # Allow Docker to access display (Linux only)

docker run -it --rm \
  --name module2-dev \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/ros2_ws:/workspace \
  physicalai/module2-gazebo:humble
```

**Windows (WSL2)**: Use [VcXsrv](https://sourceforge.net/projects/vcxsrv/) for GUI display forwarding.

#### Step 3: Verify Installation

Inside the Docker container:
```bash
# Check ROS 2
ros2 --version  # Should show: ros2 cli version X.Y.Z

# Check Gazebo
gz sim --version  # Should show Gazebo Harmonic (Fortress or Garden)

# Test launch
ros2 launch gazebo_ros gazebo.launch.py
# Gazebo GUI should open (may take 10-20 seconds first time)
```

**Troubleshooting**:
- **"Cannot connect to X server"**: Run `xhost +local:docker` on host machine
- **Slow performance**: Allocate more CPU/RAM in Docker Desktop settings
- **Gazebo crashes on startup**: Update GPU drivers or use software rendering: `LIBGL_ALWAYS_SOFTWARE=1 gz sim`

---

### Option 2: Native Installation (Ubuntu 22.04/24.04)

**Pros**: Best performance, easier GPU acceleration
**Cons**: More complex setup, potential dependency conflicts

#### Step 1: Install ROS 2 Humble

```bash
# Add ROS 2 apt repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop (full)
sudo apt update
sudo apt install -y ros-humble-desktop-full

# Install development tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update
```

#### Step 2: Install Gazebo Harmonic

```bash
# Add Gazebo repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Harmonic (Fortress or Garden)
sudo apt update
sudo apt install -y gz-harmonic

# Install ROS 2 + Gazebo integration
sudo apt install -y ros-humble-ros-gz
```

#### Step 3: Set Up Workspace

```bash
# Create ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Source ROS 2 environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Clone Module 2 code examples
cd ~/ros2_ws/src
git clone https://github.com/physicalai/module-2-digital-twin.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install
source install/setup.bash
```

---

### Unity Installation (Chapter 2 Only)

**Platform**: Windows, macOS, or Linux

#### Step 1: Install Unity Hub

1. Download Unity Hub from [unity.com/download](https://unity.com/download)
2. Install and create a free Unity account

#### Step 2: Install Unity 2022 LTS

1. Open Unity Hub
2. Go to **Installs** → **Install Editor**
3. Select **Unity 2022.3.x LTS** (latest patch version)
4. Add modules:
   - ✅ **Linux Build Support** (for building on Linux)
   - ✅ **Documentation** (offline help)

#### Step 3: Install ROS-TCP-Connector

1. Open Unity (create a new 3D project for testing)
2. Go to **Window** → **Package Manager**
3. Click **+** → **Add package from git URL**
4. Enter: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
5. Wait for installation to complete

---

## Running Your First Example

### Example 1: Spawn a Robot in Gazebo

**Goal**: Verify Gazebo + ROS 2 integration by spawning a simple robot

#### Step 1: Launch Gazebo

```bash
# Terminal 1: Start Gazebo (empty world)
source ~/ros2_ws/install/setup.bash
ros2 launch gazebo_ros gazebo.launch.py
```

**Expected**: Gazebo GUI opens with an empty gray ground plane.

#### Step 2: Spawn Robot

```bash
# Terminal 2: Spawn rigid body robot from Module 2 examples
source ~/ros2_ws/install/setup.bash
ros2 launch ch1_gazebo_physics spawn_rigid_body.launch.py
```

**Expected**:
- Blue box appears in Gazebo (at x=0, y=0, z=0.5)
- Robot falls due to gravity and bounces on ground
- Terminal shows: `[spawn_entity]: Entity 'rigid_body' spawned successfully`

#### Step 3: Verify Physics

1. In Gazebo GUI, click the **Play** button (bottom left)
2. Observe the robot falling and colliding with the ground
3. Try clicking on the robot and dragging it - physics should resist

**Troubleshooting**:
- **Robot doesn't appear**: Check terminal for URDF parsing errors
- **Robot falls through ground**: Collision meshes missing - run `check_urdf` on the URDF file
- **Gazebo freezes**: Reduce simulation real-time factor (Gazebo → Physics tab → Real-time factor = 0.5)

---

## Next Steps

### For Self-Paced Learners

1. **Complete setup validation**: Run all 3 example categories (physics, rendering, sensors)
2. **Read Chapter 1**: Start with Gazebo physics fundamentals
3. **Try exercises**: Each chapter has 3-5 hands-on exercises

### For Cohort-Based Learners (Weeks 6-7)

**Week 6**:
- Days 1-3: Chapter 1 (Gazebo Physics)
- Days 4-5: Chapter 2 Part 1 (Unity Setup)

**Week 7**:
- Days 1-2: Chapter 2 Part 2 (Unity HRI Scenarios)
- Days 3-5: Chapter 3 (Sensor Simulation)

---

## Common Issues and Solutions

<details>
<summary><strong>Issue</strong>: Gazebo crashes with "Segmentation fault"</summary>

**Cause**: Incompatible GPU drivers or missing libraries

**Solution**:
1. Update GPU drivers:
   ```bash
   # NVIDIA
   sudo ubuntu-drivers autoinstall

   # AMD
   sudo apt install -y mesa-vulkan-drivers
   ```
2. If still crashing, use software rendering:
   ```bash
   LIBGL_ALWAYS_SOFTWARE=1 gz sim
   ```
</details>

<details>
<summary><strong>Issue</strong>: Unity can't connect to ROS 2 (timeout error)</summary>

**Cause**: ROS-TCP-Endpoint not running or wrong IP address

**Solution**:
1. Start ROS-TCP-Endpoint on ROS 2 machine:
   ```bash
   ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
   ```
2. In Unity, verify **ROSConnection** GameObject settings:
   - **ROS IP Address**: `localhost` (if on same machine) or ROS 2 machine's IP
   - **ROS Port**: `10000` (default)
</details>

<details>
<summary><strong>Issue</strong>: Docker container can't display Gazebo GUI (Windows WSL2)</summary>

**Cause**: X server not running or not accessible

**Solution**:
1. Install VcXsrv on Windows
2. Launch with: "Disable access control" enabled
3. In WSL2 terminal:
   ```bash
   export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
   ```
4. Run Docker with updated DISPLAY variable
</details>

---

## Getting Help

**Documentation**:
- [ROS 2 Humble Docs](https://docs.ros.org/en/humble/)
- [Gazebo Harmonic Docs](https://gazebosim.org/docs/harmonic)
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)

**Community**:
- [ROS Discourse](https://discourse.ros.org/) - ROS 2 questions
- [Gazebo Community Forum](https://community.gazebosim.org/) - Simulation issues
- [Textbook Discussion Forum](https://github.com/physicalai/discussions) - Module-specific help

**Office Hours** (Cohort-based learners):
- Tuesdays & Thursdays, 6-7 PM UTC - Zoom link in course portal

---

## Validation Checklist

Before proceeding to Chapter 1:

- [ ] Docker or native installation complete
- [ ] Gazebo launches without errors
- [ ] Unity 2022 LTS installed (for Chapter 2)
- [ ] Module 2 code repository cloned and built
- [ ] Example robot spawns and exhibits physics behavior
- [ ] RViz2 can visualize topics (test with `/scan` from sensors)

**Estimated Setup Time**: 30-60 minutes (Docker), 60-90 minutes (native)

---

**Status**: Quickstart guide complete. Students ready to begin Chapter 1.
