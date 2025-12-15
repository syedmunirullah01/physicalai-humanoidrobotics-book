# Unity Bridge - ROS 2 Package

ROS 2 package for Unity integration via ROS-TCP-Endpoint.

## Purpose

This package provides a launch file to start the **ROS-TCP-Endpoint** server, which enables bidirectional communication between Unity and ROS 2 over TCP/IP.

## Architecture

```
Unity (C# Scripts)  <--TCP:10000-->  ROS-TCP-Endpoint (Python)  <--DDS-->  ROS 2 Nodes
```

**Data Flow**:
- **Unity → ROS 2**: Unity publishes messages via TCP → Endpoint converts to ROS 2 topics
- **ROS 2 → Unity**: ROS 2 nodes publish → Endpoint forwards via TCP to Unity subscribers

## Installation

### Prerequisites

```bash
# Ensure ROS 2 Humble is installed
source /opt/ros/humble/setup.bash

# Install ROS-TCP-Endpoint
sudo apt update
sudo apt install ros-humble-ros-tcp-endpoint
```

### Build Package

```bash
cd module-2-digital-twin/src/ch2_unity_rendering/ros2_workspace
colcon build --packages-select unity_bridge
source install/setup.bash
```

## Usage

### Starting the Bridge

```bash
# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash  # Adjust path to your workspace

# Launch Unity bridge
ros2 launch unity_bridge unity_ros_bridge.launch.py
```

**Expected Output**:
```
========================================
Unity ROS Bridge Started
========================================
ROS-TCP-Endpoint listening on: 0.0.0.0:10000

Unity Connection Instructions:
1. Open Unity project
2. Ensure ROSConnection.cs is attached to a GameObject
3. Configure IP address in Inspector (localhost or this machine's IP)
4. Play scene in Unity Editor
5. Check Unity Console for "Connected to ROS 2" message
========================================

[INFO] [ros_tcp_endpoint]: ROS-TCP Server listening on 0.0.0.0:10000
```

### Custom Configuration

**Change IP Address** (e.g., bind to specific interface):
```bash
ros2 launch unity_bridge unity_ros_bridge.launch.py ROS_IP:=192.168.1.100
```

**Change Port** (if 10000 is already in use):
```bash
ros2 launch unity_bridge unity_ros_bridge.launch.py ROS_TCP_PORT:=10001
```

(Remember to update `ROSConnection.cs` in Unity to match the new port)

## Testing Connection

### 1. Verify Server is Running

```bash
# Check if port 10000 is listening
ss -tuln | grep 10000
# Expected: tcp LISTEN 0.0.0.0:10000
```

### 2. Test from Unity

1. Open Unity project: `module-2-digital-twin/src/ch2_unity_rendering/UnityProject/`
2. Open scene: `Assets/Scenes/HumanoidHRI.unity`
3. Play scene (▶ button)
4. **Unity Console** should show: `[ROSConnection] Successfully connected to ROS 2 at localhost:10000`

### 3. Test ROS 2 → Unity Communication

**Terminal 1**: Start bridge (see above)

**Terminal 2**: Publish test message
```bash
source /opt/ros/humble/setup.bash

# Publish to joint_states (if Unity is subscribed to this topic)
ros2 topic pub --once /joint_states sensor_msgs/msg/JointState \
  "{name: ['joint1'], position: [1.57]}"
```

**Unity**: Robot joint should move to 90° (1.57 radians)

### 4. Test Unity → ROS 2 Communication

1. **Unity**: Play scene with `TeleopPublisher.cs` active
2. **Press W key** in Unity Game view (forward command)
3. **Terminal**: Check topic data
   ```bash
   ros2 topic echo /cmd_vel
   ```
4. **Expected Output**:
   ```
   linear:
     x: 0.5
     y: 0.0
     z: 0.0
   angular:
     x: 0.0
     y: 0.0
     z: 0.0
   ```

## Troubleshooting

### Issue: "Connection refused" in Unity

**Cause**: ROS-TCP-Endpoint is not running or firewall is blocking port 10000

**Solution**:
```bash
# Check if endpoint is running
ps aux | grep ros_tcp_endpoint

# If not running, launch bridge (see Usage section)

# Check firewall (Ubuntu)
sudo ufw status
sudo ufw allow 10000/tcp  # If firewall is active
```

### Issue: Unity connects but no messages received

**Cause**: Topic names don't match between Unity and ROS 2

**Solution**:
```bash
# In Unity, check topic name in JointController.cs (Inspector)
# In ROS 2, list available topics
ros2 topic list

# Ensure topic names match exactly (including leading slash)
# Example: Unity subscribes to "/joint_states" → ROS must publish to "/joint_states"
```

### Issue: Messages are delayed or stuttering

**Cause**: Network latency or TCP buffer issues

**Solution**:
1. **Use localhost** if Unity and ROS 2 are on same machine (fastest)
2. **Reduce message rate**: Publish at 10-30 Hz instead of 100+ Hz
3. **Check network**: `ping <ROS_machine_IP>` from Unity machine (should be < 10ms)

## Integration with Gazebo

You can run Unity alongside Gazebo for visualization:

**Terminal 1**: Gazebo simulation
```bash
ros2 launch gazebo_ros gazebo.launch.py
ros2 run gazebo_ros spawn_entity.py -entity robot -file robot.urdf
```

**Terminal 2**: Unity bridge
```bash
ros2 launch unity_bridge unity_ros_bridge.launch.py
```

**Unity**: Play scene → Robot in Unity mirrors Gazebo physics

## Network Configuration

### Same Machine (Unity and ROS 2 on localhost)
- **ROS_IP**: `localhost` or `127.0.0.1`
- **Unity ROSConnection.cs**: `hostName = "localhost"`

### Different Machines (Unity on Windows, ROS 2 on Linux)
- **ROS_IP**: Linux machine's network IP (e.g., `192.168.1.100`)
- **Unity ROSConnection.cs**: `hostName = "192.168.1.100"`
- **Firewall**: Allow TCP 10000 on both machines

### Docker Container (ROS 2 in Docker, Unity on host)
```bash
# Run container with --network host (Linux) or port mapping (Windows/Mac)
docker run -it --rm --network host osrf/ros:humble-desktop
# Inside container:
ros2 launch unity_bridge unity_ros_bridge.launch.py
```

## Additional Resources

- **ROS-TCP-Endpoint GitHub**: https://github.com/Unity-Technologies/ROS-TCP-Endpoint
- **Unity Robotics Hub**: https://github.com/Unity-Technologies/Unity-Robotics-Hub
- **ROS 2 + Unity Tutorial**: https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/pick_and_place/README.md

## Support

For issues:
1. Check ROS-TCP-Endpoint is running: `ros2 node list` (should show `/unity_tcp_endpoint`)
2. Verify port is listening: `ss -tuln | grep 10000`
3. Check Unity Console for connection errors
4. Review launch file output for error messages
5. Test with minimal example: `ros2 topic list` after Unity connects
