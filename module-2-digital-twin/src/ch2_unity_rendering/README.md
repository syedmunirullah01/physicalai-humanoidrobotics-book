# Chapter 2: Unity Rendering - Project Files

This directory contains Unity project files and C# scripts for Chapter 2: High-Fidelity Rendering and HRI.

## Directory Structure

```
ch2_unity_rendering/
├── UnityProject/               # Unity 2022 LTS project (create via Unity Hub)
│   ├── Assets/
│   │   ├── Scenes/             # Unity scene files
│   │   │   ├── HumanoidHRI.unity
│   │   │   └── WarehouseNavigation.unity
│   │   ├── Scripts/            # C# scripts for ROS 2 integration
│   │   │   ├── ROSConnection.cs
│   │   │   ├── JointController.cs
│   │   │   ├── TeleopPublisher.cs
│   │   │   └── HumanBehavior.cs
│   │   ├── Prefabs/            # Reusable GameObjects
│   │   │   ├── Humanoid.prefab
│   │   │   └── VirtualHuman.prefab
│   │   ├── URDF/               # Robot URDF files
│   │   │   ├── diff_drive_robot.urdf
│   │   │   └── 3dof_arm.urdf
│   │   ├── Materials/          # Rendering materials
│   │   └── Models/             # 3D meshes
│   ├── Packages/
│   │   └── manifest.json       # Unity package dependencies
│   └── ProjectSettings/        # Unity project configuration
│
└── ros2_workspace/             # ROS 2 bridge package
    └── src/
        └── unity_bridge/
            ├── package.xml
            ├── launch/
            │   └── unity_ros_bridge.launch.py
            └── README.md
```

## Setup Instructions

### Option 1: Create New Unity Project

1. **Install Unity Hub**: https://unity.com/download
2. **Install Unity 2022.3 LTS**:
   - Unity Hub → Installs → Add
   - Select **2022.3.X LTS** (latest)
   - Modules: Windows Build Support, Linux Build Support

3. **Create Project**:
   - Unity Hub → Projects → New Project
   - Template: **3D (URP)** - Universal Render Pipeline
   - Name: `Module2UnityRendering`
   - Location: `module-2-digital-twin/src/ch2_unity_rendering/UnityProject/`

4. **Install Required Packages**:
   ```
   Open Unity project
   Window → Package Manager → + → Add package from git URL

   Add these packages:
   - https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
   - https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
   ```

5. **Copy C# Scripts**:
   ```bash
   # Copy provided scripts to Unity Assets/Scripts/ folder
   cp scripts/*.cs UnityProject/Assets/Scripts/
   ```

### Option 2: Use Provided Project Template

If a pre-configured Unity project is provided:

```bash
cd module-2-digital-twin/src/ch2_unity_rendering/
# Open UnityProject/ folder in Unity Hub
```

## C# Scripts Overview

### 1. ROSConnection.cs
**Purpose**: Establish TCP connection to ROS-TCP-Endpoint

**Usage**:
- Attach to empty GameObject named `ROSManager`
- Configure IP address in Inspector (default: localhost:10000)

**Key Methods**:
- `Start()`: Connects to ROS 2 on startup

### 2. JointController.cs
**Purpose**: Subscribe to `/joint_states` and update Unity ArticulationBodies

**Usage**:
- Attach to robot root GameObject
- Assign ArticulationBody array in Inspector

**Key Methods**:
- `UpdateJointStates(JointStateMsg)`: ROS subscriber callback
- `FindJointByName(string)`: Maps ROS joint names to Unity joints

### 3. TeleopPublisher.cs
**Purpose**: Publish keyboard input to `/cmd_vel` for robot teleoperation

**Usage**:
- Attach to any GameObject (e.g., `ROSManager`)
- Configure topic name in Inspector

**Controls**:
- W/S: Forward/backward
- A/D: Turn left/right

### 4. HumanBehavior.cs
**Purpose**: Autonomous navigation for virtual humans using NavMeshAgent

**Usage**:
- Attach to virtual human GameObject
- Assign waypoint GameObjects in Inspector
- Bake NavMesh (Window → AI → Navigation → Bake)

**Key Methods**:
- `GoToNextWaypoint()`: Cycles through patrol points
- `Update()`: Updates animator based on movement speed

## ROS 2 Integration

### Starting ROS-TCP-Endpoint

On ROS 2 machine:
```bash
source /opt/ros/humble/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

Expected output:
```
[INFO] ROS-TCP Server listening on 0.0.0.0:10000
```

### Testing Connection

1. **Start ROS-TCP-Endpoint** (see above)
2. **Play Unity scene** (▶ button in Unity Editor)
3. **Check Unity Console**: Should see "Connected to ROS 2 at localhost:10000"
4. **Publish test message** from ROS:
   ```bash
   ros2 topic pub /joint_states sensor_msgs/msg/JointState "{name: ['joint1'], position: [1.57]}"
   ```
5. **Verify in Unity**: Robot joint should move to 90° (1.57 rad)

## Unity Scenes

### HumanoidHRI.unity
**Description**: Indoor office environment with humanoid robot and virtual humans

**Contents**:
- Humanoid robot prefab (imported from URDF)
- 5 virtual humans with NavMeshAgent (patrolling)
- Office furniture (desks, chairs, plants)
- Baked lightmaps for performance

**Learning Objectives**:
- Human-robot interaction scenarios
- Collision avoidance with pedestrians
- Realistic indoor lighting

### WarehouseNavigation.unity
**Description**: Warehouse environment with mobile robots and dynamic obstacles

**Contents**:
- Differential drive robot (from Chapter 1 URDF)
- 100+ crates/pallets (static batching enabled)
- Forklift robots (moving obstacles)
- NavMesh for autonomous navigation

**Learning Objectives**:
- Performance optimization (LOD, occlusion culling)
- Multi-robot coordination
- Industrial environment simulation

## Performance Optimization

### Target Metrics (GTX 1650)
- **Frame Rate**: 60+ FPS
- **Draw Calls**: < 500
- **Triangles**: < 1 million in view
- **SetPass Calls**: < 100

### Optimization Checklist
- [ ] Static batching enabled for non-moving objects
- [ ] LOD Groups on detailed models
- [ ] Occlusion culling baked (Window → Rendering → Occlusion Culling)
- [ ] Lightmaps baked for static lighting
- [ ] Real-time shadows limited to 1-2 directional lights
- [ ] GPU Instancing enabled on shared materials

### Profiling
1. **Window → Analysis → Profiler**
2. **Play scene** and monitor:
   - **CPU Usage**: Check for script bottlenecks
   - **Rendering**: Monitor draw calls, batches
   - **GPU**: Check shader complexity
3. **Target**: < 16.6ms frame time for 60 FPS

## Building for Deployment

### Windows Standalone Build
```
File → Build Settings
Platform: PC, Mac & Linux Standalone
Target Platform: Windows
Architecture: x86_64
Click Build
```

### Linux Build (for Robot Companion Computer)
```
File → Build Settings
Platform: PC, Mac & Linux Standalone
Target Platform: Linux
Architecture: x86_64
Click Build
```

### VR Build (Meta Quest 2)
```
File → Build Settings
Platform: Android
Texture Compression: ASTC
Run Device: Meta Quest 2
Click Build and Run
```

## Troubleshooting

### Unity Won't Connect to ROS
- **Check**: ROS-TCP-Endpoint is running (`ros2 run ros_tcp_endpoint default_server_endpoint`)
- **Check**: Firewall allows TCP port 10000
- **Check**: IP address in ROSConnection.cs matches ROS machine

### URDF Import Fails
- **Check**: URDF file has valid XML syntax (`check_urdf your_robot.urdf`)
- **Check**: Referenced meshes exist and are in supported format (.dae, .stl, .obj)
- **Check**: URDF Importer package is installed (Window → Package Manager)

### ArticulationBodies Explode
- **Check**: Inertia values in URDF are correct (see Chapter 1)
- **Check**: Collision geometry doesn't overlap between parent-child links
- **Fix**: Edit → Project Settings → Time → Fixed Timestep → 0.01 (smaller timestep, more stable)

### Low Frame Rate
- **Profile**: Window → Analysis → Profiler
- **Check**: Draw calls < 500
- **Optimize**: Apply LOD, occlusion culling, static batching
- **Reduce**: Real-time shadow casters, physics update rate

## Additional Resources

- **Unity Robotics Hub**: https://github.com/Unity-Technologies/Unity-Robotics-Hub
- **ROS-TCP-Connector Docs**: https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/main/README.md
- **Unity Manual**: https://docs.unity3d.com/Manual/index.html
- **C# Scripting Reference**: https://docs.unity3d.com/ScriptReference/

## Support

For issues with Unity or ROS integration:
1. Check Unity Console for error messages
2. Check ROS-TCP-Endpoint terminal output
3. Review troubleshooting section in Chapter 2 documentation
4. Post question in course discussion forum with:
   - Unity version
   - ROS 2 version
   - Error message/screenshot
   - Steps to reproduce
