# Feature Specification: Module 1 - The Robotic Nervous System (ROS 2)

**Feature Branch**: `002-ros2-nervous-system`
**Created**: 2025-12-10
**Status**: Draft
**Input**: User description: "Module 1: The Robotic Nervous System (ROS 2) - Focus: Middleware for robot control. ROS 2 Nodes, Topics, and Services. Bridging Python Agents to ROS controllers using rclpy. Understanding URDF (Unified Robot Description Format) for humanoids."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - ROS 2 Foundation Setup (Priority: P1)

A robotics student needs to understand ROS 2 fundamentals to communicate with robot systems. They will learn to install ROS 2, create their first nodes, and understand the publish-subscribe communication model that forms the nervous system of any robot.

**Why this priority**: Without understanding nodes, topics, and services, students cannot proceed to any robot control tasks. This is the foundational building block that all other modules depend on.

**Independent Test**: Can be fully tested by verifying that students can create a working ROS 2 workspace, launch multiple nodes, and demonstrate topic communication between a publisher and subscriber node using command-line tools (ros2 topic list, ros2 topic echo).

**Acceptance Scenarios**:

1. **Given** a fresh Ubuntu/Windows installation, **When** student follows installation instructions, **Then** they can successfully run `ros2 --version` and see ROS 2 Humble or Jazzy installed
2. **Given** ROS 2 is installed, **When** student creates a workspace with `ros2 pkg create`, **Then** the workspace builds without errors using `colcon build`
3. **Given** a ROS 2 workspace, **When** student creates a publisher node and subscriber node, **Then** messages flow from publisher to subscriber and can be verified with `ros2 topic echo`
4. **Given** two nodes are running, **When** student uses `ros2 node list` and `ros2 topic list`, **Then** they can identify active nodes and the topics connecting them

---

### User Story 2 - Python Integration for Agent Control (Priority: P2)

A developer building AI agents needs to control robots using Python (rclpy). They will learn to write Python nodes that can send motor commands, read sensor data, and integrate with machine learning models for autonomous behavior.

**Why this priority**: While ROS 2 CLI understanding is foundational, practical robot control requires programming. Python is the dominant language for AI/ML, making rclpy the bridge between intelligent agents and physical robots.

**Independent Test**: Can be tested by writing a Python script using rclpy that publishes velocity commands to a simulated robot and subscribes to sensor topics, demonstrating bidirectional communication without requiring URDF knowledge.

**Acceptance Scenarios**:

1. **Given** a Python environment with rclpy installed, **When** student writes a publisher node, **Then** the node successfully publishes messages at a specified rate (e.g., 10 Hz)
2. **Given** a robot simulator is running, **When** student's Python node publishes Twist messages to `/cmd_vel`, **Then** the simulated robot moves in response to velocity commands
3. **Given** sensor topics are available, **When** student's Python node subscribes to `/scan` (LIDAR data), **Then** the node receives and processes sensor messages in real-time
4. **Given** multiple Python nodes, **When** student uses ROS 2 services to request/response data, **Then** nodes communicate synchronously for actions requiring acknowledgment

---

### User Story 3 - ROS 2 Services for Request-Response Patterns (Priority: P3)

An engineer building a humanoid robot needs synchronous communication for critical actions like "pick up object" or "compute inverse kinematics." They will learn to create and call ROS 2 services that provide request-response patterns beyond pub-sub.

**Why this priority**: Many robot actions require confirmation (e.g., "Did the gripper close successfully?"). Services provide this synchronous pattern, complementing the asynchronous topic model. This is essential for reliable manipulation and control.

**Independent Test**: Can be tested independently by creating a service server that computes a simple calculation (e.g., add two numbers) and a client that calls the service, demonstrating the request-response pattern without requiring full robot integration.

**Acceptance Scenarios**:

1. **Given** a ROS 2 workspace, **When** student creates a service definition (`.srv` file), **Then** the service compiles and generates Python/C++ interfaces
2. **Given** a service server node is running, **When** student calls the service from command line using `ros2 service call`, **Then** the server processes the request and returns a response
3. **Given** a Python client node, **When** the client calls a service (e.g., to compute inverse kinematics), **Then** the client receives the response and can act on the result
4. **Given** multiple service servers, **When** student uses `ros2 service list` and `ros2 service type`, **Then** they can discover available services and their interfaces

---

### User Story 4 - Understanding URDF for Humanoid Robots (Priority: P4)

A robotics engineer needs to define the physical structure of a humanoid robot so that ROS 2 can visualize it, compute kinematics, and perform collision checking. They will learn to write URDF (Unified Robot Description Format) files that describe links, joints, and sensors.

**Why this priority**: URDF is essential for any physical robot work, but students can learn ROS 2 communication patterns without it. It becomes critical when working with actual robot models for simulation and control.

**Independent Test**: Can be tested by creating a simple URDF file for a humanoid arm (shoulder, elbow, wrist joints), loading it in RViz, and verifying that joint movements are visualized correctly using joint_state_publisher_gui.

**Acceptance Scenarios**:

1. **Given** a text editor, **When** student writes a URDF file defining a robot with 3 links and 2 joints, **Then** the URDF validates without errors using `check_urdf`
2. **Given** a valid URDF file, **When** student launches RViz and loads the robot model, **Then** the 3D visualization displays the robot structure correctly
3. **Given** a URDF with joint definitions, **When** student publishes joint states via `/joint_states` topic, **Then** RViz animates the robot moving in real-time
4. **Given** a humanoid URDF, **When** student adds sensor tags (camera, LIDAR), **Then** the sensors appear in the visualization and can be used for simulation

---

### Edge Cases

- What happens when ROS 2 nodes lose connection mid-operation (e.g., WiFi drops during remote control)? Students should learn about Quality of Service (QoS) settings to handle reliability.
- How does the system handle nodes crashing during critical operations? Students should understand ROS 2 lifecycle management and restart strategies.
- What if URDF files have circular dependencies or invalid joint limits? Students should learn to use validation tools and debugging techniques.
- How does ROS 2 handle time synchronization across distributed nodes on different machines? Students should understand the `/clock` topic and sim_time parameter.

## Requirements *(mandatory)*

### Functional Requirements

**Chapter 1: ROS 2 Installation and Workspace Setup**

- **FR-001**: Module MUST provide installation instructions for ROS 2 Humble or Jazzy on Ubuntu 22.04 and Windows (WSL2)
- **FR-002**: Students MUST be able to create a ROS 2 workspace using `colcon build` and understand workspace structure (src/, build/, install/, log/)
- **FR-003**: Module MUST explain the ROS 2 command-line interface (CLI) tools: `ros2 node`, `ros2 topic`, `ros2 service`, `ros2 param`
- **FR-004**: Students MUST successfully run example nodes from the ROS 2 tutorials (talker/listener demo)

**Chapter 2: Nodes, Topics, and Publishers/Subscribers**

- **FR-005**: Module MUST explain the ROS 2 node architecture and how nodes communicate via topics
- **FR-006**: Students MUST create a publisher node that sends messages to a topic at a specified rate
- **FR-007**: Students MUST create a subscriber node that receives and processes messages from a topic
- **FR-008**: Module MUST demonstrate Quality of Service (QoS) settings for reliable vs best-effort communication
- **FR-009**: Students MUST understand message types (std_msgs, geometry_msgs, sensor_msgs) and how to browse them using `ros2 interface show`

**Chapter 3: Python Integration with rclpy**

- **FR-010**: Module MUST provide Python examples using rclpy for all fundamental ROS 2 patterns
- **FR-011**: Students MUST write a Python node that controls a simulated robot by publishing Twist messages to `/cmd_vel`
- **FR-012**: Students MUST write a Python node that subscribes to sensor topics (`/scan` for LIDAR, `/camera/image_raw` for vision)
- **FR-013**: Module MUST explain node lifecycle (initialization, spinning, shutdown) and executor patterns
- **FR-014**: Students MUST handle exceptions and implement graceful shutdown in Python nodes

**Chapter 4: ROS 2 Services and Actions**

- **FR-015**: Module MUST explain the difference between topics (pub-sub), services (request-response), and actions (long-running tasks with feedback)
- **FR-016**: Students MUST create a custom service definition (`.srv` file) and implement a service server
- **FR-017**: Students MUST write a client that calls a service and handles the response (synchronous blocking)
- **FR-018**: Module MUST demonstrate async service calls for non-blocking operations
- **FR-019**: Students MUST understand when to use services vs topics (e.g., "Should I use a topic or service for sensor data?" answer: topic for continuous streams, service for one-time queries)

**Chapter 5: URDF and Robot Description**

- **FR-020**: Module MUST explain URDF structure: links, joints, visual, collision, and inertial elements
- **FR-021**: Students MUST create a simple URDF file for a humanoid arm with 3 degrees of freedom (shoulder, elbow, wrist)
- **FR-022**: Module MUST demonstrate loading URDF into RViz and visualizing joint movements
- **FR-023**: Students MUST add sensors (camera, LIDAR) to URDF and understand Gazebo sensor plugins
- **FR-024**: Module MUST explain coordinate frames (base_link, world, sensor frames) and the TF tree
- **FR-025**: Students MUST validate URDF files using `check_urdf` and debug common errors (missing links, invalid joint types)

### Key Entities

- **ROS 2 Node**: Computational process that performs a specific task (e.g., motor controller, sensor processor, planner). Nodes communicate via topics, services, and actions.
- **Topic**: Named bus for asynchronous message passing. Publishers send messages, subscribers receive them. Multiple publishers/subscribers can connect to the same topic.
- **Service**: Synchronous request-response communication pattern. Client sends request, server processes and returns response. Used for queries and actions requiring confirmation.
- **Message**: Data structure passed between nodes (e.g., Twist for velocity, LaserScan for LIDAR data, Image for camera). Defined in `.msg` files.
- **URDF (Unified Robot Description Format)**: XML-based format describing robot structure (links, joints, sensors). Used for visualization (RViz), simulation (Gazebo), and kinematics.
- **Link**: Rigid body in URDF (e.g., upper arm, forearm, hand). Has visual, collision, and inertial properties.
- **Joint**: Connection between two links defining motion (revolute for rotation, prismatic for sliding, fixed for rigid connections). Has limits and dynamics.
- **ROS 2 Workspace**: Directory structure containing source code (src/), build artifacts (build/), and installed packages (install/). Created with `colcon build`.

## Success Criteria *(mandatory)*

### Measurable Outcomes

**Chapter 1: Installation and Setup**

- **SC-001**: 95% of students successfully install ROS 2 and create a working workspace within 30 minutes
- **SC-002**: Students can list active nodes and topics using CLI tools without referencing documentation

**Chapter 2: Topics and Communication**

- **SC-003**: Students create a publisher-subscriber pair that exchanges at least 10 messages per second
- **SC-004**: Students demonstrate understanding of QoS by explaining when to use "reliable" vs "best effort" settings

**Chapter 3: Python Integration**

- **SC-005**: Students write a Python node that successfully controls a simulated robot's movement (forward, backward, turn)
- **SC-006**: Students implement error handling that gracefully shuts down nodes when Ctrl+C is pressed
- **SC-007**: 90% of students complete the Python integration exercises without requiring instructor debugging assistance

**Chapter 4: Services**

- **SC-008**: Students create a working service server-client pair that exchanges custom messages
- **SC-009**: Students correctly identify 3 scenarios where services are more appropriate than topics

**Chapter 5: URDF**

- **SC-010**: Students create a valid URDF file for a 3-DOF humanoid arm that loads in RViz without errors
- **SC-011**: Students add a camera sensor to their URDF and verify it appears in the TF tree
- **SC-012**: Students debug a broken URDF file by identifying missing parent links or invalid joint types within 10 minutes

### Qualitative Outcomes

- Students understand that ROS 2 is a communication middleware, not a programming language
- Students can explain the pub-sub pattern and why it's preferred for continuous data streams
- Students feel confident navigating ROS 2 documentation and finding message definitions
- Students recognize when they need synchronous (service) vs asynchronous (topic) communication
- Students can visualize the difference between a robot's physical structure (URDF) and its behavior (control code)

## Assumptions

- Students have basic Python programming knowledge (functions, classes, loops)
- Students have access to a computer with Ubuntu 22.04 or Windows 11 with WSL2
- Students have completed the textbook preface and understand the three-tier hardware structure (Tier A: simulation-only is sufficient for Module 1)
- ROS 2 Humble or Jazzy is the target distribution (as specified in the preface)
- Students have basic Linux command-line familiarity (cd, ls, mkdir)
- Simulated robots are provided (TurtleBot3 or similar) for testing Python control code
- Each chapter builds sequentially: cannot do URDF (Chapter 5) without understanding topics (Chapter 2)

## Out of Scope

- Advanced ROS 2 features (parameters, lifecycle nodes, component composition) - covered in later modules
- ROS 1 vs ROS 2 migration - this textbook assumes students start with ROS 2
- Multi-robot systems and namespace management - covered in Module 3 (AI Brain)
- Real-time control and deterministic behavior - covered in Module 3 (Navigation & Control)
- URDF xacro macros and parameterization - simplified URDF taught first, xacro introduced later
- Gazebo simulation in depth - introduced lightly in Module 1, covered fully in Module 2 (Digital Twin)
- MoveIt 2 (motion planning) - covered in Module 3
- Custom message compilation from .msg files - students use standard messages first (std_msgs, geometry_msgs)

## Dependencies

- ROS 2 Humble or Jazzy installed (installation covered in Chapter 1)
- Python 3.8+ with rclpy package
- Colcon build tool for workspace management
- RViz for URDF visualization
- Basic simulator (TurtleBot3 Gazebo simulation or equivalent) for Python control exercises
- urdf_tutorial and joint_state_publisher_gui packages for URDF exercises
- Constitution Principle III (Bloom's Taxonomy): Module follows remember → understand → apply progression
- Constitution Principle IX (Module Structure): This is Module 1 of 4, forms foundation for all subsequent modules

## Open Questions

None - this specification is complete based on the provided description. The module structure is clear:
- Chapter 1: Installation/Setup (foundation)
- Chapter 2: Topics/Pub-Sub (core communication)
- Chapter 3: Python/rclpy (practical programming)
- Chapter 4: Services (request-response pattern)
- Chapter 5: URDF (robot description)

All learning outcomes map to Bloom's Taxonomy levels (remember ROS 2 commands, understand pub-sub pattern, apply Python to robot control, analyze when to use services vs topics).
