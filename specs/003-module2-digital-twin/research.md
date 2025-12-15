# Research & Technical Decisions: Module 2 - The Digital Twin

**Feature**: Module 2 - The Digital Twin (Gazebo & Unity)
**Date**: 2025-12-12
**Status**: Completed

## Overview

This document captures research findings and technical decisions for Module 2's simulation platforms, integration approaches, and content delivery patterns. All decisions prioritize educational effectiveness, three-tier hardware accessibility, and alignment with robotics industry standards.

---

## Decision 1: Gazebo Version Selection

### Question
Should we use Gazebo Harmonic (modern ROS 2 native) or Gazebo Classic (legacy but widely documented)?

### Research Findings

**Gazebo Harmonic (formerly Ignition Gazebo)**:
- Native ROS 2 support via `ros_gz` packages
- Modern architecture with component-based plugin system
- Better performance (multithreading, improved physics solvers)
- SDF (Simulation Description Format) as primary world format
- Active development and long-term support from Open Robotics

**Gazebo Classic (Gazebo 11)**:
- Mature ecosystem with extensive tutorials
- URDF-centric workflow (familiar to Module 1 graduates)
- Larger plugin library (but many ported to Harmonic)
- ROS 2 support via compatibility layer (less native)
- In maintenance mode (no major new features)

**Educational Context**:
- Students completing Module 1 have URDF knowledge
- Industry trend: New projects adopting Gazebo Harmonic
- Learning curve: Harmonic requires SDF understanding but teaches modern standards

### Decision

**Use Gazebo Harmonic as primary platform**

### Rationale

1. **Future-proof skills**: Gazebo Harmonic is the long-term path for ROS 2 robotics
2. **Better ROS 2 integration**: Native support reduces troubleshooting friction for students
3. **Performance**: Students with mid-range hardware benefit from optimized physics
4. **Module progression**: Prepares for NVIDIA Isaac Sim (Module 3) which has similar modern architecture

### Alternatives Considered

- **Gazebo Classic**: Rejected due to legacy status and inferior ROS 2 support
- **Webots**: Excellent simulator but smaller community and different workflow
- **PyBullet**: Good for algorithmic work but limited HRI/visualization features

### Implementation Notes

- Chapter 1 will include URDF-to-SDF conversion primer
- Docker image provides Gazebo Harmonic + ROS 2 Humble pre-configured
- Fallback tutorials for Gazebo Classic compatibility (appendix only)

---

## Decision 2: Unity ROS Integration

### Question
Which Unity-ROS 2 bridge should we recommend: ROS-TCP-Connector or Unity Robotics Hub?

### Research Findings

**ROS-TCP-Connector**:
- Official Unity package maintained by Unity Technologies
- TCP-based communication (no ROS 2 DDS dependency in Unity)
- Cross-platform (Windows/Linux/macOS Unity development)
- Active development as of 2024-2025
- Examples focused on pick-and-place and navigation

**Unity Robotics Hub** (broader ecosystem):
- Includes ROS-TCP-Connector plus URDF Importer and additional tools
- Well-documented tutorials and GitHub repos
- Larger community and third-party examples
- Better suited for complete robotics workflows

**Direct ROS 2 DDS Bridge**:
- Native ROS 2 integration (same DDS as Linux robots)
- Platform-limited (requires ROS 2 installation on Unity machine)
- More complex setup for Windows/macOS students

### Decision

**Use ROS-TCP-Connector as part of Unity Robotics Hub**

### Rationale

1. **Cross-platform accessibility**: Students on Windows can participate (Tier A requirement)
2. **Official support**: Unity Technologies maintains package, reducing risk of abandonment
3. **Simplified setup**: TCP bridge easier than DDS configuration for beginners
4. **Complete workflow**: Robotics Hub includes URDF importer needed for Chapter 2

### Alternatives Considered

- **Direct DDS bridge**: Rejected due to platform limitations and complexity
- **Custom socket solution**: Not justified given official package availability
- **ROS#**: Community project but less active than Unity Robotics Hub

### Implementation Notes

- Install via Unity Package Manager (Git URL)
- TCP bridge runs on ROS 2 machine, Unity connects remotely
- Latency acceptable for HRI visualization (not real-time control)
- Chapter 2 includes Unity/ROS troubleshooting section (common connection issues)

---

## Decision 3: Sensor Simulation Platform

### Question
Should sensor examples (LiDAR, depth cameras, IMUs) use Gazebo plugins, Unity plugins, or both?

### Research Findings

**Gazebo Sensors**:
- High-fidelity physics-based ray tracing for LiDAR
- Realistic noise models and configurable parameters
- Direct ROS 2 topic publishing (native integration)
- Better for algorithm validation (perception pipeline testing)

**Unity Sensors**:
- GPU-accelerated rendering (faster for complex scenes)
- Easier to create photorealistic environments for depth cameras
- Good for visualization and HRI scenarios
- ROS 2 integration via ROS-TCP-Connector (additional latency)

**Real Hardware Considerations**:
- Intel RealSense D435i (Tier B) outputs similar to Gazebo depth camera
- RPLiDAR A1 (Tier B) matches Gazebo 2D LiDAR specs
- IMU data format identical (sensor_msgs/Imu)

### Decision

**Use Gazebo for sensor simulation (Chapter 3 primary), Unity for visualization (Chapter 2 supplement)**

### Rationale

1. **Physics accuracy**: Gazebo ray-casting matches real sensor behavior better
2. **ROS 2 native**: Direct topic publishing reduces learning overhead
3. **Tier A completeness**: Gazebo sensors work without Unity installation
4. **Sim-to-real transfer**: Gazebo sensor data format matches Tier B hardware

### Alternatives Considered

- **Unity-only sensors**: Rejected due to less accurate physics and additional bridge complexity
- **Dual implementation**: Considered but adds maintenance burden and confuses students
- **NVIDIA Isaac Sim sensors**: Reserved for Module 3 (advanced simulation)

### Implementation Notes

- Chapter 3 focuses on Gazebo sensor plugins (gazebo_ros_ray_sensor, gazebo_ros_imu)
- Chapter 2 demonstrates Unity depth camera for HRI visualization only
- Sensor configuration files (YAML) portable between Gazebo and real hardware
- Noise models calibrated to match RealSense D435i and RPLiDAR A1 specs

---

## Decision 4: URDF Inertia Tensor Calculation

### Question
How should students calculate correct inertia tensors for stable physics simulation?

### Research Findings

**Manual Calculation**:
- Formula-based for simple geometries (box, cylinder, sphere)
- Prone to errors for complex shapes
- Educational value in understanding mass distribution

**MeshLab + Python Scripts**:
- Import STL/OBJ meshes
- Calculate moments of inertia from triangle meshes
- Requires additional tool installation

**SolidWorks/Fusion 360 Export**:
- CAD software provides accurate inertia tensors
- Not accessible to all students (proprietary software)
- Best for Tier C students with physical robot CAD models

**`check_urdf` Validation**:
- ROS 2 tool to verify URDF syntax and physics sanity
- Warns about unrealistic inertia values
- Does not calculate tensors, only validates

### Decision

**Provide formula-based manual calculation for simple shapes + validation with `check_urdf`**

### Rationale

1. **Pedagogical value**: Students learn physics concepts (moment of inertia, principal axes)
2. **Zero dependencies**: No additional software beyond ROS 2 tools
3. **Tier A accessibility**: Works in Docker environment
4. **Sufficient for Module 2**: Simple geometries (boxes, cylinders) cover 80% of examples

### Alternatives Considered

- **MeshLab automation**: Adds complexity, deferred to "Advanced Extensions" section
- **CAD software requirement**: Violates Tier A accessibility principle
- **Hardcoded values without explanation**: Misses learning opportunity

### Implementation Notes

- Chapter 1 includes inertia tensor calculation worksheet (box, cylinder, sphere formulas)
- Provide Python script to generate URDF `<inertial>` blocks from mass + geometry
- `check_urdf` validation step in every example workflow
- "Advanced Extensions" section references MeshLab for complex meshes

---

## Decision 5: Mermaid Diagram Patterns

### Question
What standardized Mermaid diagram templates should be used for ROS 2 computation graphs, TF trees, and sensor pipelines?

### Research Findings

**ROS 2 Documentation Standards**:
- Official ROS 2 docs use `graph TD` for node/topic graphs
- Node names in rectangles, topics in rounded boxes
- Data flow direction: top-to-bottom or left-to-right

**Accessibility Requirements** (Constitution Principle IV):
- Semantic node labels (not generic node_1, node_2)
- Text-based alternative in `<details>` block
- Preceding paragraph explaining diagram purpose

**Docusaurus Mermaid Plugin**:
- Supports graph, sequenceDiagram, stateDiagram-v2
- Renders inline in MDX with syntax highlighting
- Mobile-responsive (scales to narrow screens)

### Decision

**Adopt standardized Mermaid templates for 4 diagram types**

1. **ROS 2 Computation Graph**: `graph TD` with nodes as rectangles, topics as rounded boxes
2. **TF Tree**: `graph TD` with coordinate frames and parent-child relationships
3. **Sensor Data Flow**: `sequenceDiagram` for Gazebo → Plugin → ROS 2 → RViz2 pipeline
4. **Unity ROS Integration**: `graph LR` showing bidirectional TCP communication

### Rationale

1. **Consistency**: Students recognize patterns across chapters
2. **Accessibility**: Text alternatives meet WCAG 2.1 AA standards
3. **Reusability**: Templates serve as starting point for student projects
4. **Visual learning**: Reduces cognitive load vs. text-only explanations

### Implementation Notes

- Create `mermaid-templates/` directory in docs repo with reusable snippets
- Each diagram type has accompanying accessibility checklist
- Chapter authors use template + customize node labels
- Example text alternative:

```markdown
<details>
<summary>Text alternative for screen readers</summary>

This diagram shows the ROS 2 computation graph for TurtleBot3 navigation:
- /turtlebot3_node publishes to /cmd_vel topic (geometry_msgs/Twist)
- /lidar_node publishes to /scan topic (sensor_msgs/LaserScan)
- /navigation_node subscribes to /scan and publishes to /cmd_vel
</details>
```

---

## Technology Stack Summary

| Component | Decision | Version/Package |
|-----------|----------|-----------------|
| **Physics Simulation** | Gazebo Harmonic | Gazebo Harmonic (Fortress/Garden) |
| **ROS 2 Integration** | ros_gz packages | ROS 2 Humble/Jazzy |
| **Unity Version** | Unity 2022 LTS | 2022.3.x (latest patch) |
| **Unity-ROS Bridge** | ROS-TCP-Connector | Unity Robotics Hub 0.7+ |
| **Sensor Simulation** | Gazebo plugins | gazebo_ros_ray_sensor, gazebo_ros_imu, gazebo_ros_camera |
| **URDF Tools** | check_urdf | ROS 2 core tools |
| **Diagrams** | Mermaid.js | Docusaurus Mermaid plugin |
| **Content Format** | Docusaurus MDX | Docusaurus 3.x |

---

## Open Questions (None)

All research topics resolved with clear decisions. Ready to proceed to Phase 1 (data model and contracts).

---

## References

1. [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic/getstarted)
2. [Unity Robotics Hub GitHub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
3. [ROS 2 Humble + Gazebo Integration Guide](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
4. [Mermaid.js Syntax Reference](https://mermaid.js.org/intro/)
5. [URDF Inertia Calculation Tutorial](http://wiki.ros.org/urdf/Tutorials/Adding%20Physical%20and%20Collision%20Properties%20to%20a%20URDF%20Model)
6. [WCAG 2.1 AA Accessibility Guidelines](https://www.w3.org/WAI/WCAG21/quickref/)

---

**Status**: All technical decisions finalized. Proceeding to Phase 1 design artifacts.
