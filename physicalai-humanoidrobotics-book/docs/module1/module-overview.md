---
id: module-overview
title: "Module 1: The Robotic Nervous System (ROS 2)"
sidebar_label: "Module Overview"
sidebar_position: 1
description: "Introduction to ROS 2 fundamentals - the middleware nervous system for robot control"
keywords: [ros2, middleware, nodes, topics, services, urdf, rclpy, gazebo, robot operating system]
---

# Module 1: The Robotic Nervous System (ROS 2)

## Welcome to Module 1

This module teaches you the foundational middleware layer that every robot depends on: **ROS 2 (Robot Operating System 2)**. Think of ROS 2 as the nervous system of a robot - it enables different components (sensors, motors, planning algorithms, AI models) to communicate seamlessly, just like neurons transmitting signals in a biological brain.

By the end of this module, you'll understand how to:
- Install and configure ROS 2 environments
- Create nodes that communicate via topics and services
- Write Python code to control simulated robots
- Define robot structures using URDF

**Duration**: Weeks 3-5 of 13-week course (3 weeks total)
**Prerequisites**: Python basics, Linux command line, Docker fundamentals

---

## Prerequisites Checklist

Before starting Module 1, ensure you have:

### Required Knowledge
- [ ] **Python 3**: Variables, functions, classes, loops, exception handling
- [ ] **Linux CLI**: `cd`, `ls`, `mkdir`, file permissions, environment variables
- [ ] **Command-line tools**: Package managers (apt, pip), text editors (nano, vim, or VS Code)
- [ ] **Version control**: Basic git commands (`git clone`, `git commit`, `git push`)

### Required Software
- [ ] **Operating System**: Ubuntu 22.04/24.04 **OR** Windows 11 with WSL2
- [ ] **Python**: Python 3.10 or later installed
- [ ] **Docker** (optional but recommended): For consistent environment
- [ ] **Internet connection**: For downloading ROS 2 packages

### Recommended (Not Required)
- IDE with Python support (VS Code, PyCharm)
- Basic understanding of pub-sub patterns
- Familiarity with XML or YAML file formats

:::tip Need a Refresher?
If you're missing any prerequisites, check **Weeks 1-2** of the course (Foundations module) for Python, Linux, and Docker tutorials. You can also proceed and learn as you go - we provide starter code and detailed explanations.
:::

---

## 5-Chapter Roadmap

### **Chapter 1: ROS 2 Installation and Workspace Setup** (Week 3, Days 1-2)
**What You'll Learn**: Install ROS 2, create your first workspace, understand the CLI tools
**Key Concepts**: `colcon build`, workspace structure (src/, build/, install/), CLI commands (`ros2 node`, `ros2 topic`)
**Hands-On**: Verify installation, create workspace, run talker/listener demo
**Time**: 1.5-2 hours reading + exercises

### **Chapter 2: Nodes, Topics, and Publishers/Subscribers** (Week 3, Days 3-5)
**What You'll Learn**: ROS 2 communication architecture, pub-sub pattern, Quality of Service (QoS)
**Key Concepts**: Nodes, topics, publishers, subscribers, message types, decoupling
**Hands-On**: Write minimal publisher/subscriber nodes, experiment with QoS settings
**Time**: 2-2.5 hours reading + exercises

### **Chapter 3: Python Integration with rclpy** (Week 4, Days 1-3)
**What You'll Learn**: Control robots using Python, read sensor data, handle exceptions gracefully
**Key Concepts**: rclpy library, Twist messages, LaserScan processing, node lifecycle
**Hands-On**: Control TurtleBot3 with keyboard teleoperation, detect obstacles with LIDAR
**Time**: 2-2.5 hours reading + exercises

### **Chapter 4: ROS 2 Services and Actions** (Week 4, Days 4-5)
**What You'll Learn**: Synchronous request-response communication, when to use services vs topics
**Key Concepts**: Service definitions (.srv files), service servers/clients, async calls
**Hands-On**: Create AddTwoInts service, call services from CLI and Python
**Time**: 1.5-2 hours reading + exercises

### **Chapter 5: URDF and Robot Description** (Week 5, Days 1-3)
**What You'll Learn**: Define robot structure for visualization and kinematics
**Key Concepts**: URDF syntax (links, joints), RViz visualization, TF trees, sensor integration
**Hands-On**: Create 3-DOF humanoid arm URDF, visualize in RViz, debug broken URDFs
**Time**: 2-2.5 hours reading + exercises

### **Integration Project: ROS 2 Navigation Pipeline** (Week 5, Days 4-5)
**Capstone**: Combine all concepts - build a robot navigation system using topics, services, and URDF
**Time**: 4-6 hours

---

## Learning Objectives (Bloom's Taxonomy)

By completing Module 1, you will be able to:

### Remember & Understand
- Explain what ROS 2 is and why it's used for robot middleware
- Describe the pub-sub communication pattern
- Identify when to use topics vs. services vs. actions

### Apply
- Install ROS 2 and create workspaces with `colcon build`
- Write Python nodes using rclpy to publish and subscribe to messages
- Control simulated robots by publishing Twist commands
- Load and visualize URDF robot models in RViz

### Analyze & Evaluate
- Choose appropriate QoS settings for different scenarios (reliable vs. best-effort)
- Decide when to use synchronous (services) vs. asynchronous (topics) communication
- Debug broken URDF files using validation tools

### Create
- Build custom ROS 2 packages with correct `package.xml` dependencies
- Design URDF models for humanoid robot arms with joints and sensors
- Implement exception handling and graceful shutdown in Python nodes

---

## What You Will Build

### By End of Chapter 1
- Working ROS 2 Humble/Jazzy installation
- ROS 2 workspace with src/, build/, install/ structure
- Ability to run and inspect demo nodes with CLI tools

### By End of Chapter 2
- Minimal publisher node (publishes String messages at 10 Hz)
- Subscriber node that receives and processes messages
- Understanding of QoS profiles (Reliable, BestEffort, SensorData)

### By End of Chapter 3
- Keyboard teleoperation node for TurtleBot3 (w/a/s/d controls)
- Obstacle detection node using LIDAR data
- Exception handling patterns for graceful shutdown

### By End of Chapter 4
- Custom service definition (AddTwoInts.srv)
- Service server that processes requests
- Async service client that doesn't block node execution

### By End of Chapter 5
- 3-DOF humanoid arm URDF (shoulder, elbow, wrist)
- RViz visualization with joint_state_publisher_gui
- Camera sensor integrated into URDF with TF tree

### By End of Integration Project
- Complete navigation pipeline combining all concepts
- Robot that moves, avoids obstacles, and responds to service commands

---

## Tools & Technologies

### Core Tools (Required)
- **ROS 2 Humble** or **ROS 2 Jazzy**: Middleware framework
- **rclpy**: Python client library for ROS 2
- **colcon**: Build tool for ROS 2 workspaces
- **Gazebo Harmonic**: Open-source robot simulator (Tier A - simulation only)

### Visualization & Debugging
- **RViz**: 3D visualization for robots and sensor data
- **ros2 CLI tools**: `ros2 node list`, `ros2 topic echo`, `ros2 service call`
- **check_urdf**: URDF validation tool

### Optional (For Later Modules)
- **Isaac Sim**: Advanced simulator (introduced in Module 2)
- **MoveIt 2**: Motion planning (introduced in Module 3)
- **Nav2**: Navigation stack (introduced in Module 3)

---

## Success Criteria

You'll know you've mastered Module 1 when you can:

- [ ] Install ROS 2 and create a workspace within 30 minutes (SC-001)
- [ ] List active nodes and topics using CLI without documentation (SC-002)
- [ ] Create publisher-subscriber pair exchanging 10+ messages/second (SC-003)
- [ ] Explain when to use "reliable" vs "best effort" QoS (SC-004)
- [ ] Write Python node that controls simulated robot movement (SC-005)
- [ ] Implement error handling that gracefully shuts down on Ctrl+C (SC-006)
- [ ] Create working service server-client pair (SC-008)
- [ ] Identify 3 scenarios where services beat topics (SC-009)
- [ ] Create valid 3-DOF humanoid arm URDF that loads in RViz (SC-010)
- [ ] Add camera sensor to URDF and verify in TF tree (SC-011)
- [ ] Debug broken URDF within 10 minutes (SC-012)

---

## Study Tips

### For Beginners
1. **Start with Chapter 1**: Don't skip the installation chapter even if you think you know ROS. ROS 2 is different from ROS 1.
2. **Use CLI first**: Master `ros2 topic list`, `ros2 node info` before writing code. CLI tools are your debugging lifeline.
3. **Copy starter code**: We provide templates - modify them gradually instead of writing from scratch.
4. **Run examples frequently**: Test every 5-10 lines of code. Don't write entire programs before testing.

### For Experienced Developers
1. **Focus on ROS 2 differences**: If you know ROS 1, pay attention to DDS, QoS, and component architecture changes.
2. **Skim Chapter 1**: If you're already proficient with Linux, skim installation and focus on workspace structure.
3. **Challenge yourself**: Skip guided exercises, go straight to open-ended exercises.
4. **Contribute**: Share your URDF models or Python node templates with the community.

### For All Learners
- **Read code out loud**: Understand every line - don't just copy-paste.
- **Break when stuck**: If you're debugging for >15 minutes, take a break or ask for help.
- **Visualize with diagrams**: Draw the pub-sub graph on paper before coding.
- **Test incrementally**: Publisher works? Great. Now add subscriber. Then add QoS. Build step-by-step.

---

## Hardware Requirements (Three-Tier System)

This module uses **Tier A (Simulation Only)** - you do NOT need physical hardware.

### Tier A: Simulation Only ($0) - **Used in This Module**
- **What**: Gazebo Harmonic simulation environment
- **Cost**: Free, open-source
- **Hardware**: Any laptop with 8GB RAM, Ubuntu 22.04/24.04 or WSL2
- **Use Case**: Learning ROS 2 fundamentals without physical robots

### Tier B: Edge AI ($700) - Introduced in Module 2
- **What**: NVIDIA Jetson Orin Nano for on-device AI processing
- **Cost**: ~$700 (Jetson + accessories)
- **Use Case**: Running neural networks on robot hardware

### Tier C: Physical Robots ($10,000+) - Introduced in Module 3
- **What**: Unitree G1, Fourier GR1, or other humanoid platforms
- **Cost**: $10,000-$16,000
- **Use Case**: Full physical robot deployment

:::info
**Module 1 is 100% simulation-based**. You can complete all exercises in Gazebo without spending any money on hardware. Physical robot integration comes in later modules.
:::

---

## Next Steps

Ready to begin? Start with **Chapter 1: ROS 2 Installation and Workspace Setup** →

Have questions? Check the FAQ or join the community Discord.

**Estimated Time to Complete Module 1**: 15-20 hours (reading + exercises + integration project)

Let's build the nervous system of your first robot! 🤖
