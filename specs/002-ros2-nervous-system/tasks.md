---
description: "Implementation tasks for Module 1: The Robotic Nervous System (ROS 2)"
---

# Tasks: Module 1 - The Robotic Nervous System (ROS 2)

**Input**: Design documents from `specs/002-ros2-nervous-system/`
**Prerequisites**: plan.md (Phase 0 Research + Phase 1 Design complete), spec.md (4 user stories)

**Tests**: This feature does not require automated tests. Success validation occurs through readability analysis, comprehension quizzes, and code example execution in Gazebo simulation.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each learning module.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus content**: `physicalai-humanoidrobotics-book/docs/module1/`
- **ROS 2 code examples**: `module-1-ros2-fundamentals/src/`
- **Urdu translations**: `physicalai-humanoidrobotics-book/i18n/ur/docusaurus-plugin-content-docs/current/module1/`
- **Shared templates**: `module-1-ros2-fundamentals/src/textbook_templates/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and directory structure for Module 1 content

- [X] T001 Create module1/ directory in physicalai-humanoidrobotics-book/docs/module1/
- [X] T002 Create separate ROS 2 code repository module-1-ros2-fundamentals/ with src/, launch/, tests/, docker/ folders
- [X] T003 [P] Update Docusaurus sidebars.ts to add Module 1 category with 7 items (overview + 5 chapters + integration project)
- [X] T004 [P] Create i18n/glossary.yml entries for ROS 2 technical terms (node, topic, service, URDF, QoS, rclpy, colcon)
- [X] T005 [P] Create Docker devcontainer in module-1-ros2-fundamentals/docker/Dockerfile.humble with ROS 2 Humble pinned version
- [X] T006 [P] Create package.xml template in module-1-ros2-fundamentals/src/textbook_templates/ for reusable ROS 2 package structure

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Module-level content and shared templates that ALL chapters depend on

**⚠️ CRITICAL**: No chapter work can begin until this phase is complete

- [X] T007 Create module-overview.md in physicalai-humanoidrobotics-book/docs/module1/module-overview.md with prerequisites checklist, 5-chapter roadmap, learning objectives (maps to all 4 user stories)
- [X] T008 [P] Create Python node template (Node subclass, create_publisher, create_subscription, destroy_node) in module-1-ros2-fundamentals/src/textbook_templates/minimal_node_template.py
- [X] T009 [P] Create launch file template (robot_state_publisher, RViz, Gazebo spawning) in module-1-ros2-fundamentals/launch/template.launch.py
- [X] T010 [P] Configure CI pipeline in module-1-ros2-fundamentals/.github/workflows/ for ruff linting, pytest, launch tests
- [X] T011 [P] Create readability validation script or document validation process using textstat (FK 13-15, FRE 30-50) - reference existing scripts/validate-readability.py

**Checkpoint**: Foundation ready - chapter implementation can now begin in parallel

---

## Phase 3: User Story 1 - ROS 2 Foundation Setup (Priority: P1) 🎯 MVP

**Goal**: Students install ROS 2, create workspace, understand CLI tools, run talker/listener demo, verify topic communication

**Independent Test**: Student creates workspace with `colcon build`, launches talker/listener nodes, verifies messages with `ros2 topic echo /chatter`, lists nodes with `ros2 node list`

**Covers**: Chapters 1 (Installation & Workspace) + Chapter 2 (Nodes, Topics, Pub-Sub)

### Implementation for User Story 1

- [X] T012 [P] [US1] Write chapter1-installation.mdx in physicalai-humanoidrobotics-book/docs/module1/chapter1-installation.mdx with frontmatter, learning outcomes (FR-001 to FR-004)
- [X] T013 [P] [US1] Write chapter2-topics.mdx in physicalai-humanoidrobotics-book/docs/module1/chapter2-topics.mdx with frontmatter, learning outcomes (FR-005 to FR-009)
- [X] T014 [US1] Add installation instructions section to chapter1-installation.mdx for Ubuntu 22.04, Ubuntu 24.04, Windows WSL2 (FR-001)
- [X] T015 [US1] Add workspace creation section to chapter1-installation.mdx explaining colcon build, src/, build/, install/, log/ structure (FR-002)
- [ ] T016 [US1] Add ROS 2 CLI tools reference section to chapter1-installation.mdx with table of ros2 node, topic, service, param commands (FR-003)
- [ ] T017 [US1] Add troubleshooting section to chapter1-installation.mdx for missing dependencies, source setup.bash errors (FR-004 support)
- [ ] T018 [US1] Create Exercise 1 section in chapter1-installation.mdx: "Install ROS 2 and Verify Installation" (guided, 20 min, CLI-only)
- [ ] T019 [US1] Create Exercise 2 section in chapter1-installation.mdx: "Create Your First Workspace" (guided, 15 min, mkdir + colcon build)
- [ ] T020 [US1] Create Exercise 3 section in chapter1-installation.mdx: "Explore Talker/Listener Demo with CLI Tools" (semi-guided, 25 min, ros2 topic echo)
- [ ] T021 [US1] Add Mermaid diagram to chapter1-installation.mdx showing workspace directory structure (graph TD: src/, build/, install/, log/) with semantic description
- [ ] T022 [US1] Add ROS 2 node architecture section to chapter2-topics.mdx explaining pub-sub pattern, decoupling, many-to-many communication (FR-005)
- [ ] T023 [US1] Add message types section to chapter2-topics.mdx with table of std_msgs, geometry_msgs, sensor_msgs and ros2 interface show usage (FR-009)
- [ ] T024 [US1] Add QoS profiles section to chapter2-topics.mdx explaining Reliable, BestEffort, SensorData profiles with use case table (FR-008)
- [ ] T025 [US1] Create Exercise 1 section in chapter2-topics.mdx: "Write a Minimal Publisher Node" (guided, 30 min, Python with starter code)
- [ ] T026 [US1] Create Exercise 2 section in chapter2-topics.mdx: "Write a Subscriber Node and Verify with CLI" (guided, 30 min, Python with starter code)
- [ ] T027 [US1] Create Exercise 3 section in chapter2-topics.mdx: "Experiment with QoS Settings" (semi-guided, 40 min, Python without starter code)
- [ ] T028 [US1] Add Mermaid computation graph to chapter2-topics.mdx showing talker node → /chatter topic → listener node (graph TD with semantic description)
- [ ] T029 [US1] Add Mermaid sequence diagram to chapter2-topics.mdx showing message flow timeline: talker publishes → topic buffers → listener receives
- [ ] T030 [P] [US1] Create ch1_workspace_demo/ ROS 2 package in module-1-ros2-fundamentals/src/ch1_workspace_demo/ with package.xml (no code, just workspace structure demo)
- [ ] T031 [P] [US1] Create minimal_publisher.py in module-1-ros2-fundamentals/src/ch2_talker_listener/minimal_publisher.py (rclpy Node subclass, publishes std_msgs/String at 10 Hz)
- [ ] T032 [P] [US1] Create minimal_subscriber.py in module-1-ros2-fundamentals/src/ch2_talker_listener/minimal_subscriber.py (rclpy callback pattern, subscribes to /chatter)
- [ ] T033 [P] [US1] Create qos_profile_examples.py in module-1-ros2-fundamentals/src/ch2_talker_listener/qos_profile_examples.py (demonstrates Reliable, BestEffort, SensorData QoS)
- [ ] T034 [P] [US1] Create package.xml for ch2_talker_listener package with rclpy, std_msgs dependencies in module-1-ros2-fundamentals/src/ch2_talker_listener/package.xml
- [ ] T035 [US1] Run readability validation on chapter1-installation.mdx using textstat (FK 13-15, FRE 30-50), revise if needed
- [ ] T036 [US1] Run readability validation on chapter2-topics.mdx using textstat (FK 13-15, FRE 30-50), revise if needed
- [ ] T037 [US1] Test all Chapter 1 + Chapter 2 code examples in Gazebo simulation (minimal_publisher.py, minimal_subscriber.py, qos_profile_examples.py must run without errors)
- [ ] T038 [US1] Extract technical terms from chapter1-installation.mdx and chapter2-topics.mdx to i18n/glossary.yml (add entries like "ROS 2 Workspace", "colcon", "Topic", "Publisher", "Subscriber", "QoS")

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently - students can install ROS 2, create workspace, run pub-sub examples, understand CLI tools

---

## Phase 4: User Story 2 - Python Integration for Agent Control (Priority: P2)

**Goal**: Students write Python nodes using rclpy to control simulated robot (publish Twist to /cmd_vel), subscribe to LIDAR (/scan), implement exception handling

**Independent Test**: Python script publishes velocity commands to TurtleBot3 in Gazebo, robot moves forward/backward/turns. Second script subscribes to /scan and prints minimum distance. Both handle Ctrl+C gracefully.

**Covers**: Chapter 3 (Python Integration with rclpy)

### Implementation for User Story 2

- [ ] T039 [US2] Write chapter3-python.mdx in physicalai-humanoidrobotics-book/docs/module1/chapter3-python.mdx with frontmatter, learning outcomes (FR-010 to FR-014)
- [ ] T040 [US2] Add rclpy Node subclass pattern section to chapter3-python.mdx explaining best practices vs. old-style rclpy.create_node (FR-010)
- [ ] T041 [US2] Add Twist message section to chapter3-python.mdx explaining linear.x/y/z and angular.x/y/z structure for robot control (FR-011 support)
- [ ] T042 [US2] Add LaserScan message section to chapter3-python.mdx explaining ranges array, angle_min/max/increment for LIDAR data (FR-012 support)
- [ ] T043 [US2] Add node lifecycle section to chapter3-python.mdx explaining __init__, spin(), KeyboardInterrupt, destroy_node() (FR-013)
- [ ] T044 [US2] Add exception handling section to chapter3-python.mdx with try/finally pattern, graceful shutdown best practices (FR-014)
- [ ] T045 [US2] Add executor pattern section to chapter3-python.mdx explaining single-threaded vs. multi-threaded executors (FR-013 support)
- [ ] T046 [US2] Create Exercise 1 section in chapter3-python.mdx: "Control TurtleBot3 with Keyboard Teleoperation Node" (guided, 45 min, Python with starter code, publish Twist to /cmd_vel)
- [ ] T047 [US2] Create Exercise 2 section in chapter3-python.mdx: "Obstacle Detection from LIDAR Data" (semi-guided, 50 min, Python without starter code, subscribe to /scan, stop if obstacle <0.5m)
- [ ] T048 [US2] Create Exercise 3 section in chapter3-python.mdx: "Exception Handling and Graceful Shutdown" (open-ended, 30 min, modify previous exercises to handle Ctrl+C)
- [ ] T049 [US2] Add Mermaid sequence diagram to chapter3-python.mdx showing node lifecycle: __init__ → create_subscription/publisher → spin() → KeyboardInterrupt → destroy_node()
- [ ] T050 [US2] Add Mermaid graph to chapter3-python.mdx showing sensor pipeline: /scan topic → LaserScan callback → distance calculation → /cmd_vel topic (stop command)
- [ ] T051 [P] [US2] Create turtlebot3_teleop.py in module-1-ros2-fundamentals/src/ch3_robot_control/turtlebot3_teleop.py (keyboard control: w=forward, s=backward, a=left, d=right, publishes Twist)
- [ ] T052 [P] [US2] Create lidar_obstacle_detector.py in module-1-ros2-fundamentals/src/ch3_robot_control/lidar_obstacle_detector.py (subscribes to /scan, detects obstacle <0.5m, publishes stop command to /cmd_vel)
- [ ] T053 [P] [US2] Create graceful_shutdown.py in module-1-ros2-fundamentals/src/ch3_robot_control/graceful_shutdown.py (exception handling template with try/finally, destroy_node() example)
- [ ] T054 [P] [US2] Create package.xml for ch3_robot_control package with rclpy, geometry_msgs, sensor_msgs dependencies in module-1-ros2-fundamentals/src/ch3_robot_control/package.xml
- [ ] T055 [P] [US2] Create turtlebot3.launch.py in module-1-ros2-fundamentals/launch/turtlebot3.launch.py to spawn TurtleBot3 in Gazebo for teleoperation testing
- [ ] T056 [US2] Run readability validation on chapter3-python.mdx using textstat (FK 13-15, FRE 30-50), revise if needed
- [ ] T057 [US2] Test all Chapter 3 code examples in Gazebo with TurtleBot3 (turtlebot3_teleop.py, lidar_obstacle_detector.py, graceful_shutdown.py must run without errors, robot must respond to commands)
- [ ] T058 [US2] Extract technical terms from chapter3-python.mdx to i18n/glossary.yml (add entries like "rclpy", "Twist", "LaserScan", "Node Lifecycle", "Executor")

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently - students can write Python nodes to control robots and process sensor data

---

## Phase 5: User Story 3 - ROS 2 Services for Request-Response Patterns (Priority: P3)

**Goal**: Students understand topics vs. services vs. actions, create custom .srv files, implement service server/client, call services from CLI and Python async

**Independent Test**: Service server computes simple calculation (add two integers), client calls service from command line with `ros2 service call`, Python async client calls service without blocking node spin

**Covers**: Chapter 4 (ROS 2 Services and Actions)

### Implementation for User Story 3

- [ ] T059 [US3] Write chapter4-services.mdx in physicalai-humanoidrobotics-book/docs/module1/chapter4-services.mdx with frontmatter, learning outcomes (FR-015 to FR-019)
- [ ] T060 [US3] Add topics vs. services vs. actions comparison section to chapter4-services.mdx with table explaining asynchronous vs. synchronous vs. long-running task patterns (FR-015)
- [ ] T061 [US3] Add service definition syntax section to chapter4-services.mdx explaining .srv file structure (request fields --- response fields) (FR-016 support)
- [ ] T062 [US3] Add when to use services section to chapter4-services.mdx with decision criteria: one-time queries, confirmation-required actions, request-response pattern (FR-019)
- [ ] T063 [US3] Add when NOT to use services section to chapter4-services.mdx explaining continuous data streams (use topics), long-running tasks (use actions - deferred to Module 3) (FR-019 support)
- [ ] T064 [US3] Add ros2 service call command-line usage section to chapter4-services.mdx with examples (FR-017 support)
- [ ] T065 [US3] Create Exercise 1 section in chapter4-services.mdx: "Define and Compile Custom Service" (guided, 25 min, create AddTwoInts.srv, update package.xml, colcon build)
- [ ] T066 [US3] Create Exercise 2 section in chapter4-services.mdx: "Implement Service Server (Add Two Integers)" (guided, 35 min, Python with starter code, test with ros2 service call)
- [ ] T067 [US3] Create Exercise 3 section in chapter4-services.mdx: "Write Asynchronous Service Client" (semi-guided, 40 min, Python without starter code, async call with future.result())
- [ ] T068 [US3] Create Exercise 4 section in chapter4-services.mdx: "Decision Analysis: Topics vs. Services" (quiz, 15 min, 3 scenarios provided, student explains which pattern to use)
- [ ] T069 [US3] Add Mermaid sequence diagram to chapter4-services.mdx showing service request-response flow: Client sends request → Server processes → Server sends response → Client receives
- [ ] T070 [US3] Add Mermaid comparison diagram to chapter4-services.mdx showing Topics (asynchronous, many-to-many) vs. Services (synchronous, one-to-one)
- [ ] T071 [P] [US3] Create AddTwoInts.srv service definition in module-1-ros2-fundamentals/src/ch4_service_examples/srv/AddTwoInts.srv (request: int64 a, int64 b --- response: int64 sum)
- [ ] T072 [P] [US3] Create add_two_ints_server.py in module-1-ros2-fundamentals/src/ch4_service_examples/add_two_ints_server.py (service server implementation, receives two ints, returns sum)
- [ ] T073 [P] [US3] Create add_two_ints_client.py in module-1-ros2-fundamentals/src/ch4_service_examples/add_two_ints_client.py (synchronous blocking client, calls service, prints result)
- [ ] T074 [P] [US3] Create add_two_ints_client_async.py in module-1-ros2-fundamentals/src/ch4_service_examples/add_two_ints_client_async.py (asynchronous non-blocking client with future.result())
- [ ] T075 [P] [US3] Create package.xml for ch4_service_examples package with rclpy dependencies and rosidl_default_generators for .srv compilation in module-1-ros2-fundamentals/src/ch4_service_examples/package.xml
- [ ] T076 [P] [US3] Create CMakeLists.txt for ch4_service_examples package to compile AddTwoInts.srv service definition in module-1-ros2-fundamentals/src/ch4_service_examples/CMakeLists.txt
- [ ] T077 [US3] Run readability validation on chapter4-services.mdx using textstat (FK 13-15, FRE 30-50), revise if needed
- [ ] T078 [US3] Test all Chapter 4 code examples (AddTwoInts service must compile with colcon build, add_two_ints_server.py must run, ros2 service call must work, async client must not block)
- [ ] T079 [US3] Extract technical terms from chapter4-services.mdx to i18n/glossary.yml (add entries like "Service", "Request-Response", ".srv File", "Service Server", "Service Client", "Asynchronous Call")

**Checkpoint**: At this point, User Stories 1, 2, AND 3 should all work independently - students understand when to use services vs. topics and can implement both patterns

---

## Phase 6: User Story 4 - Understanding URDF for Humanoid Robots (Priority: P4)

**Goal**: Students create 3-DOF humanoid arm URDF with links/joints/limits, load in RViz, visualize joint movements, add camera sensor, debug broken URDF

**Independent Test**: URDF file for humanoid arm (shoulder, elbow, wrist) validates with check_urdf, loads in RViz, joint_state_publisher_gui moves joints, camera sensor appears in TF tree

**Covers**: Chapter 5 (URDF and Robot Description)

### Implementation for User Story 4

- [ ] T080 [US4] Write chapter5-urdf.mdx in physicalai-humanoidrobotics-book/docs/module1/chapter5-urdf.mdx with frontmatter, learning outcomes (FR-020 to FR-025)
- [ ] T081 [US4] Add URDF structure section to chapter5-urdf.mdx explaining <robot>, <link>, <joint>, <visual>, <collision>, <inertial> elements (FR-020)
- [ ] T082 [US4] Add joint types section to chapter5-urdf.mdx explaining revolute, prismatic, fixed, continuous, floating, planar with use cases (FR-020 support)
- [ ] T083 [US4] Add joint limits section to chapter5-urdf.mdx explaining <limit lower upper effort velocity> with examples (FR-021 support)
- [ ] T084 [US4] Add TF tree section to chapter5-urdf.mdx explaining coordinate frames, robot_state_publisher, transforms between links (FR-024)
- [ ] T085 [US4] Add Gazebo sensor plugins section to chapter5-urdf.mdx explaining libgazebo_ros_camera.so, libgazebo_ros_ray_sensor.so (LIDAR) (FR-023)
- [ ] T086 [US4] Add check_urdf validation section to chapter5-urdf.mdx explaining command-line tool usage, common error messages (FR-025)
- [ ] T087 [US4] Add common URDF errors section to chapter5-urdf.mdx listing missing parent link, circular dependencies, joint axis not normalized, invalid joint types (FR-025 support)
- [ ] T088 [US4] Create Exercise 1 section in chapter5-urdf.mdx: "Write URDF for 3-DOF Humanoid Arm" (semi-guided, 60 min, 4 links + 3 revolute joints with limits, starter code provided)
- [ ] T089 [US4] Create Exercise 2 section in chapter5-urdf.mdx: "Visualize URDF in RViz with Joint State Publisher" (guided, 30 min, launch file with robot_state_publisher + joint_state_publisher_gui + rviz2)
- [ ] T090 [US4] Create Exercise 3 section in chapter5-urdf.mdx: "Add Camera Sensor to URDF" (semi-guided, 45 min, add camera_link and <sensor> tag, verify in TF tree)
- [ ] T091 [US4] Create Exercise 4 section in chapter5-urdf.mdx: "Debug Broken URDF" (open-ended, 20 min, provided URDF with 3 intentional errors: missing parent, invalid joint type, incorrect axis)
- [ ] T092 [US4] Add Mermaid URDF structure diagram to chapter5-urdf.mdx showing hierarchy: base_link → upper_arm → forearm → hand (graph TD with parent-child relationships)
- [ ] T093 [US4] Add Mermaid TF tree diagram to chapter5-urdf.mdx showing coordinate frames: world → base_link → shoulder_link → elbow_link → wrist_link → camera_link (graph TD)
- [ ] T094 [P] [US4] Create humanoid_arm_3dof.urdf in module-1-ros2-fundamentals/src/ch5_urdf_models/urdf/humanoid_arm_3dof.urdf (complete URDF with 4 links, 3 revolute joints, visual/collision/inertial properties)
- [ ] T095 [P] [US4] Create display.launch.py in module-1-ros2-fundamentals/launch/display.launch.py (RViz launch file with robot_state_publisher, joint_state_publisher_gui, loads humanoid_arm_3dof.urdf)
- [ ] T096 [P] [US4] Create gazebo_arm.launch.py in module-1-ros2-fundamentals/launch/gazebo_arm.launch.py (spawn humanoid_arm_3dof.urdf in Gazebo with sensor plugins)
- [ ] T097 [P] [US4] Create broken_urdf_debug.urdf in module-1-ros2-fundamentals/src/ch5_urdf_models/urdf/broken_urdf_debug.urdf (intentionally broken URDF for Exercise 4: missing parent link, invalid joint type "invalid_type", incorrect joint axis <axis xyz="1 0 0 0"/>)
- [ ] T098 [P] [US4] Create package.xml for ch5_urdf_models package with urdf, robot_state_publisher, joint_state_publisher_gui dependencies in module-1-ros2-fundamentals/src/ch5_urdf_models/package.xml
- [ ] T099 [US4] Run readability validation on chapter5-urdf.mdx using textstat (FK 13-15, FRE 30-50), revise if needed
- [ ] T100 [US4] Test all Chapter 5 code examples (humanoid_arm_3dof.urdf must validate with check_urdf, load in RViz without errors, joints move with joint_state_publisher_gui, Gazebo spawning works)
- [ ] T101 [US4] Verify broken_urdf_debug.urdf intentionally fails check_urdf with clear error messages for debugging exercise
- [ ] T102 [US4] Extract technical terms from chapter5-urdf.mdx to i18n/glossary.yml (add entries like "URDF", "Link", "Joint", "Revolute Joint", "TF Tree", "robot_state_publisher", "check_urdf")

**Checkpoint**: All user stories should now be independently functional - students can install ROS 2, write Python nodes, implement services, and create URDF models

---

## Phase 7: Integration & Polish

**Purpose**: Module-level integration project, cross-cutting improvements, translation

- [ ] T103 Create integration-project.md in physicalai-humanoidrobotics-book/docs/module1/integration-project.md with Week 5 capstone project: "Build a ROS 2 navigation pipeline combining topics, services, and URDF" (integrates all 4 user stories)
- [ ] T104 [P] Update Docusaurus sidebars.ts to finalize Module 1 category order: module-overview → chapter1 → chapter2 → chapter3 → chapter4 → chapter5 → integration-project
- [ ] T105 [P] Run markdown-link-check on all Module 1 MDX files (chapter1 through chapter5, module-overview, integration-project) to ensure zero dead links
- [ ] T106 [P] Run Lighthouse accessibility audit on Docusaurus site, ensure all Mermaid diagrams have semantic descriptions and alt-text, Accessibility score >90
- [ ] T107 Create ADR for architecturally significant decisions made during Module 1 implementation (e.g., "ADR-002: Why Gazebo Harmonic over Isaac Sim for Module 1", "ADR-003: Why colcon build over catkin_make")
- [ ] T108 [P] Tag all Module 1 chapters for Urdu translation in physicalai-humanoidrobotics-book/i18n/ur/docusaurus-plugin-content-docs/current/module1/ (create directory structure, copy MDX files, add translation flags)
- [ ] T109 [P] Review i18n/glossary.yml for completeness - verify all technical terms from Module 1 have English and Urdu translations with context and definitions
- [ ] T110 Final end-to-end test: fresh environment (Docker container with ROS 2 Humble), student follows module-overview prerequisites → completes all 5 chapters → runs integration project → all code examples work in Gazebo

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3 → P4)
- **Integration & Polish (Phase 7)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1) - Foundation Setup**: Can start after Foundational (Phase 2) - No dependencies on other stories - Covers Chapter 1 + Chapter 2
- **User Story 2 (P2) - Python Integration**: Can start after Foundational (Phase 2) - Logically builds on US1 concepts but technically independent (can test Python nodes without understanding CLI from Chapter 1) - Covers Chapter 3
- **User Story 3 (P3) - Services**: Can start after Foundational (Phase 2) - Logically builds on US1 (topics) and US2 (Python) but technically independent (service examples don't require robot control or URDF) - Covers Chapter 4
- **User Story 4 (P4) - URDF**: Can start after Foundational (Phase 2) - Logically builds on US1 (understanding topics for /joint_states) but technically independent (URDF can be created and validated without writing Python code) - Covers Chapter 5

**NOTE**: While user stories are *technically* independent (can be tested separately), the *pedagogical* order is sequential (students should learn Chapter 1 → 2 → 3 → 4 → 5). Tasks can be implemented in parallel by different content authors, but chapters should be published in order.

### Within Each User Story

- MDX chapter files can be written in parallel (marked [P])
- Code examples can be created in parallel (marked [P])
- Content sections must be written before readability validation
- Code examples must be created before testing in Gazebo
- Readability validation and testing happens after all content/code complete
- Glossary extraction happens after chapter content finalized

### Parallel Opportunities

- All Setup tasks (T001-T006) marked [P] can run in parallel
- All Foundational tasks (T008-T011) marked [P] can run in parallel
- Once Foundational phase completes, all 4 user stories (Phases 3-6) can start in parallel:
  - Developer A: User Story 1 (Chapters 1-2)
  - Developer B: User Story 2 (Chapter 3)
  - Developer C: User Story 3 (Chapter 4)
  - Developer D: User Story 4 (Chapter 5)
- Within each user story:
  - Writing chapter MDX files [P]
  - Creating code examples [P]
  - Package.xml and CMakeLists.txt files [P]
- All Integration & Polish tasks (T104-T109) marked [P] can run in parallel

---

## Parallel Example: User Story 1

```bash
# Launch all chapter writing tasks for User Story 1 together:
Task T012 [P]: "Write chapter1-installation.mdx"
Task T013 [P]: "Write chapter2-topics.mdx"

# Launch all code example creation tasks together:
Task T030 [P]: "Create ch1_workspace_demo/ package"
Task T031 [P]: "Create minimal_publisher.py"
Task T032 [P]: "Create minimal_subscriber.py"
Task T033 [P]: "Create qos_profile_examples.py"
Task T034 [P]: "Create package.xml for ch2_talker_listener"

# Then sequentially: T014-T029 (content sections), T035-T036 (readability), T037 (testing), T038 (glossary)
```

---

## Parallel Example: User Story 2

```bash
# Launch all code example creation tasks for User Story 2 together:
Task T051 [P]: "Create turtlebot3_teleop.py"
Task T052 [P]: "Create lidar_obstacle_detector.py"
Task T053 [P]: "Create graceful_shutdown.py"
Task T054 [P]: "Create package.xml for ch3_robot_control"
Task T055 [P]: "Create turtlebot3.launch.py"

# Then sequentially: T039-T050 (content sections), T056 (readability), T057 (testing), T058 (glossary)
```

---

## Parallel Example: User Story 3

```bash
# Launch all code example creation tasks for User Story 3 together:
Task T071 [P]: "Create AddTwoInts.srv"
Task T072 [P]: "Create add_two_ints_server.py"
Task T073 [P]: "Create add_two_ints_client.py"
Task T074 [P]: "Create add_two_ints_client_async.py"
Task T075 [P]: "Create package.xml for ch4_service_examples"
Task T076 [P]: "Create CMakeLists.txt for ch4_service_examples"

# Then sequentially: T059-T070 (content sections), T077 (readability), T078 (testing), T079 (glossary)
```

---

## Parallel Example: User Story 4

```bash
# Launch all code example creation tasks for User Story 4 together:
Task T094 [P]: "Create humanoid_arm_3dof.urdf"
Task T095 [P]: "Create display.launch.py"
Task T096 [P]: "Create gazebo_arm.launch.py"
Task T097 [P]: "Create broken_urdf_debug.urdf"
Task T098 [P]: "Create package.xml for ch5_urdf_models"

# Then sequentially: T080-T093 (content sections), T099 (readability), T100-T101 (testing), T102 (glossary)
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T006)
2. Complete Phase 2: Foundational (T007-T011) - CRITICAL - blocks all stories
3. Complete Phase 3: User Story 1 (T012-T038)
4. **STOP and VALIDATE**: Test User Story 1 independently
   - Student can install ROS 2 and create workspace (Chapter 1)
   - Student can write publisher/subscriber and understand topics (Chapter 2)
   - All code examples run in Gazebo without errors
   - Readability metrics pass (FK 13-15, FRE 30-50)
5. Deploy/demo if ready (Chapters 1-2 published to Docusaurus site)

### Incremental Delivery

1. Complete Setup + Foundational (Phases 1-2) → Foundation ready
2. Add User Story 1 (Phase 3) → Test independently → Deploy/Demo (MVP: Chapters 1-2 published!)
3. Add User Story 2 (Phase 4) → Test independently → Deploy/Demo (Chapters 1-3 published)
4. Add User Story 3 (Phase 5) → Test independently → Deploy/Demo (Chapters 1-4 published)
5. Add User Story 4 (Phase 6) → Test independently → Deploy/Demo (All 5 chapters published)
6. Add Integration Project (Phase 7) → Complete Module 1 → Final deployment

Each story adds value without breaking previous stories. Students can start learning from partial module while remaining chapters are in progress.

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together (Phases 1-2)
2. Once Foundational is done:
   - **Developer A**: User Story 1 (Chapters 1-2, 27 tasks T012-T038)
   - **Developer B**: User Story 2 (Chapter 3, 20 tasks T039-T058)
   - **Developer C**: User Story 3 (Chapter 4, 21 tasks T059-T079)
   - **Developer D**: User Story 4 (Chapter 5, 23 tasks T080-T102)
3. Stories complete and integrate independently
4. Team reconvenes for Integration & Polish (Phase 7)

---

## Task Summary

**Total Tasks**: 110 tasks
- **Phase 1 (Setup)**: 6 tasks
- **Phase 2 (Foundational)**: 5 tasks (BLOCKING - must complete before any user story)
- **Phase 3 (User Story 1)**: 27 tasks (Chapters 1-2: Installation, Topics)
- **Phase 4 (User Story 2)**: 20 tasks (Chapter 3: Python/rclpy)
- **Phase 5 (User Story 3)**: 21 tasks (Chapter 4: Services)
- **Phase 6 (User Story 4)**: 23 tasks (Chapter 5: URDF)
- **Phase 7 (Integration & Polish)**: 8 tasks

**Parallel Opportunities**: 48 tasks marked [P] can run in parallel within their phase/story

**Independent Test Criteria**:
- **US1**: Student creates workspace, runs talker/listener, verifies with CLI tools
- **US2**: Python script controls TurtleBot3 movement and processes LIDAR data
- **US3**: Service server-client pair exchanges custom messages
- **US4**: 3-DOF humanoid arm URDF loads in RViz with joint movements

**Suggested MVP Scope**: Phase 1 (Setup) + Phase 2 (Foundational) + Phase 3 (User Story 1) = **38 tasks** for minimal viable module (Chapters 1-2 only)

**Quality Gates**:
- Readability: FK 13-15, FRE 30-50 (validated per chapter)
- Code quality: All examples run in Gazebo Tier A simulation
- Accessibility: Lighthouse score >90, all Mermaid diagrams have semantic descriptions
- Link validation: Zero dead links (markdown-link-check on all MDX files)

---

## Notes

- [P] tasks = different files, no dependencies within phase/story
- [Story] label maps task to specific user story for traceability (US1, US2, US3, US4)
- Each user story should be independently completable and testable
- Tests are NOT included (educational content validated through readability analysis, comprehension quizzes, code execution)
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
- All code examples MUST be Tier A (Gazebo simulation) - no hardware dependencies
- All content MUST meet FK 13-15 readability target (university undergraduate level)
- Glossary terms extracted incrementally as chapters are written (English first, Urdu translation in Phase 7)
