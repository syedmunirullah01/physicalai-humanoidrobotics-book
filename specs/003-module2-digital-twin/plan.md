# Implementation Plan: Module 2 - The Digital Twin (Gazebo & Unity)

**Branch**: `003-module2-digital-twin` | **Date**: 2025-12-12 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-module2-digital-twin/spec.md`

## Summary

Create Module 2 of the Physical AI & Humanoid Robotics textbook focusing on physics simulation (Gazebo Harmonic), high-fidelity rendering (Unity 2022 LTS), and sensor simulation (LiDAR, depth cameras, IMUs). This module consists of 3 chapters with Docusaurus MDX content, code examples in a companion repository, and Mermaid diagrams for visual learning. Students learn to build digital twins that simulate robot behavior before deploying to physical hardware, supporting the three-tier accessibility model (Simulation/Edge AI/Physical Robot).

## Technical Context

**Content Format**: Docusaurus 3.x MDX (Markdown + JSX components)
**Code Examples**: Python 3.10+ (rclpy for ROS 2), C# (Unity scripts)
**Primary Dependencies**:
- Gazebo Harmonic + ROS 2 Humble/Jazzy integration
- Unity 2022 LTS + ROS-TCP-Connector
- Docusaurus plugins: `@docusaurus/plugin-content-docs`, Mermaid support

**Storage**:
- Content: Git repository (`physicalai-humanoidrobotics-book/docs/module2/`)
- Code examples: Git repository (`module-2-digital-twin/` - similar to `module-1-ros2-fundamentals/`)
- Assets: Static images, videos, 3D meshes in `static/assets/module2/`

**Testing**:
- Content: Readability validation (Flesch-Kincaid Grade 13-15)
- Code: pytest for ROS 2 examples, Unity Test Framework for C# scripts
- Integration: Launch tests for Gazebo worlds, ROS-Unity communication

**Target Platform**:
- Documentation: Web (GitHub Pages / Vercel)
- Code execution: Linux (Ubuntu 22.04/24.04) for Gazebo, Windows/Linux/macOS for Unity

**Project Type**: Educational content module with companion code repository

**Performance Goals**:
- Content readability: Flesch-Kincaid Grade 13-15, Reading Ease 30-50
- Code execution: Gazebo simulations maintain 30+ FPS with moderate complexity robots
- Unity rendering: 30+ FPS for HRI scenarios on mid-range GPUs (GTX 1650+)

**Constraints**:
- **Safety-first**: All motor control code must include E-Stop documentation
- **Tier A accessibility**: All examples must work in simulation without hardware
- **Mermaid diagrams**: Required for physics pipelines, TF trees, sensor data flow
- **No implementation leaks**: Docusaurus content must not expose React/build details to students

**Scale/Scope**:
- 3 chapters (6,000-8,000 words each, ~20,000 words total)
- 10-15 code examples per chapter (ROS 2 launch files, URDF models, Unity scenes)
- 5-7 Mermaid diagrams per chapter
- 3-5 exercises per chapter with validation criteria

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ I. Three-Tier Hardware Accessibility

**Status**: PASS

- **Tier A (Simulation)**: Complete coverage - all examples use Gazebo Harmonic and Unity without hardware dependencies
- **Tier B (Edge AI)**: Simulation-to-real transfer concepts covered in Chapter 3 (sensor calibration, noise modeling)
- **Tier C (Physical Robots)**: Optional extensions mentioned but not required

**Evidence**: Spec SC-009 states "Learning objectives are achievable using alternative simulation environments (Isaac Sim, Webots) with minimal adaptation"

### ✅ II. Safety-First Physical AI

**Status**: PASS

- **E-Stop Documentation**: Chapter 1 (Gazebo physics) will include E-Stop procedures for simulation testing
- **Danger Admonitions**: Motor control examples will have `:::danger` blocks warning about force application
- **Simulation-First Testing**: No physical robot code introduced until simulation tests pass (Tier A → Tier C progression)

**Evidence**: FR-005 requires "explain how to debug common physics issues" including safety constraints

### ✅ III. Bloom's Taxonomy Learning Outcomes

**Status**: PASS

- **Chapter 1 (Analyze)**: Students deconstruct physics simulations to understand mass, inertia, collision effects
- **Chapter 2 (Apply)**: Students use Unity tools to build HRI scenarios following provided patterns
- **Chapter 3 (Create)**: Students design custom sensor configurations and validation pipelines

**Evidence**: Success criteria SC-001 requires "end-to-end simulation project combining physics, rendering, and sensor simulation" (Create level)

### ✅ IV. Visual Intelligence (Mermaid-First Diagrams)

**Status**: PASS - Will create:

- **Physics Pipeline**: `graph LR` showing URDF → Physics Engine → Collision Detection → Force Application
- **TF Trees**: `graph TD` for sensor coordinate frames (base_link → camera_link → lidar_link)
- **Sensor Data Flow**: `sequenceDiagram` showing Gazebo Plugin → ROS 2 Topic → RViz2 Visualization
- **Unity ROS Integration**: `graph TD` showing ROS-TCP-Connector bidirectional communication

**Accessibility**: Each diagram will have preceding description and `<details>` text alternative

### ✅ V. RAG Chatbot Integration

**Status**: PASS (Infrastructure level - no module-specific work)

- Module 2 content will be indexed into Qdrant for semantic search
- Selected text Q&A feature applies to all simulation chapters
- No assessment questions flagged in this module (exercises are open-ended)

**Evidence**: Content structure compatible with existing RAG pipeline

### ✅ VI. Personalization Engine

**Status**: PASS - Content adapts based on:

- **Hardware Tier**: Tier A users see simulation-only examples; Tier B/C see hardware comparison notes
- **Experience Level**: Beginners get "Prerequisites" callouts for URDF/ROS 2 review; Experts see "Advanced Extensions" for Isaac Sim integration
- **Pacing**: Self-paced learners see all optional exercises; Cohort-based see recommended subset

**Evidence**: Spec assumptions state "Students have completed Module 1" - content builds on existing knowledge

### ✅ VII. Bilingual Support (English/Urdu)

**Status**: PASS

- English content as source of truth in `docs/module2/`
- Urdu translations in `/i18n/ur/docusaurus-plugin-content-docs/current/module2/`
- Technical terms (Gazebo, Unity, LiDAR, IMU) remain in English per glossary conventions
- Code comments remain in English; explanatory prose fully translated

**Evidence**: Constitution Principle VII translation workflow applies to all modules

### ✅ VIII. Reusable Intelligence Architecture

**Status**: PASS

- Mermaid diagram templates reusable across chapters (physics, TF trees)
- ROS 2 launch file patterns from Module 1 extended for Gazebo sensors
- Unity ROS integration code examples serve as templates for Module 4 (Isaac Sim)

**Evidence**: Code examples will follow `module-1-ros2-fundamentals/` repository structure for consistency

### ✅ IX. Module-Aligned Content Structure

**Status**: PASS

- **Module 2 Timeline**: Weeks 6-7 (2 weeks total)
- **Chapter 1**: Week 6, Days 1-3 (Gazebo physics - 3 days)
- **Chapter 2**: Week 6, Days 4-5 + Week 7, Days 1-2 (Unity rendering - 4 days)
- **Chapter 3**: Week 7, Days 3-5 (Sensor simulation - 3 days)

**Evidence**: Spec assumptions state "6-8 hours per chapter" aligning with 2-week module schedule

### ✅ X. Code Quality Standards

**Status**: PASS

- **package.xml**: All ROS 2 code examples include complete `package.xml` with dependencies
- **rclpy patterns**: Following Module 1 conventions (minimal_publisher/subscriber patterns)
- **No dead links**: CI validation with `markdown-link-check` on all MDX files
- **Reproducible environments**: Docker devcontainer for Gazebo + ROS 2 Humble, Unity project with locked package versions

**Evidence**: FR-005 requires "Code examples provided in the module execute without errors on standard ROS 2 Humble + Gazebo Harmonic + Unity 2022 LTS setup"

**Constitution Compliance Summary**: 10/10 principles PASS. No violations requiring justification.

## Project Structure

### Documentation (this feature)

```text
specs/003-module2-digital-twin/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification (completed)
├── research.md          # Phase 0 output - Technical decisions for simulation platforms
├── data-model.md        # Phase 1 output - Learning content entities
├── quickstart.md        # Phase 1 output - Getting started with Module 2
├── contracts/           # Phase 1 output - Content structure contracts
│   ├── chapter-structure.md      # Standard chapter template
│   ├── code-example-format.md    # ROS 2 and Unity example structure
│   └── mermaid-diagram-spec.md   # Diagram requirements and patterns
├── checklists/          # Quality validation
│   └── requirements.md  # Specification quality checklist (completed)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

This feature creates educational content in two locations:

#### 1. Docusaurus Content (`physicalai-humanoidrobotics-book/`)

```text
physicalai-humanoidrobotics-book/
├── docs/
│   └── module2/                  # Module 2 documentation
│       ├── module-overview.mdx   # Module introduction
│       ├── chapter1-gazebo-physics.mdx        # Chapter 1: Physics simulation
│       ├── chapter2-unity-rendering.mdx       # Chapter 2: Unity HRI
│       ├── chapter3-sensor-simulation.mdx     # Chapter 3: LiDAR/Depth/IMU
│       └── integration-project.mdx            # End-to-end capstone project
├── static/
│   └── assets/
│       └── module2/              # Images, videos, 3D models
│           ├── diagrams/         # Mermaid diagram source (backup)
│           ├── screenshots/      # Gazebo/Unity UI screenshots
│           └── meshes/           # Sample URDF collision meshes
├── sidebars.ts                   # Update with Module 2 entries
└── docusaurus.config.ts          # No changes needed (Mermaid already configured)
```

#### 2. Code Examples Repository (`module-2-digital-twin/`)

```text
module-2-digital-twin/               # New repository (parallel to module-1-ros2-fundamentals/)
├── src/
│   ├── ch1_gazebo_physics/       # Chapter 1 code examples
│   │   ├── package.xml
│   │   ├── launch/
│   │   │   ├── spawn_rigid_body.launch.py
│   │   │   ├── wheeled_robot.launch.py
│   │   │   └── articulated_arm.launch.py
│   │   ├── urdf/
│   │   │   ├── rigid_body.urdf
│   │   │   ├── diff_drive_robot.urdf
│   │   │   └── 3dof_arm.urdf
│   │   └── worlds/
│   │       ├── empty_world.sdf
│   │       └── obstacle_course.sdf
│   ├── ch2_unity_rendering/      # Chapter 2 code examples
│   │   ├── UnityProject/         # Unity 2022 LTS project
│   │   │   ├── Assets/
│   │   │   │   ├── Scenes/
│   │   │   │   │   ├── HumanoidHRI.unity
│   │   │   │   │   └── WarehouseNavigation.unity
│   │   │   │   ├── Scripts/
│   │   │   │   │   ├── ROSConnection.cs
│   │   │   │   │   ├── JointController.cs
│   │   │   │   │   └── HumanBehavior.cs
│   │   │   │   └── Prefabs/
│   │   │   │       ├── Humanoid.prefab
│   │   │   │       └── VirtualHuman.prefab
│   │   │   ├── Packages/
│   │   │   │   └── manifest.json  # ROS-TCP-Connector dependency
│   │   │   └── ProjectSettings/
│   │   └── ros2_workspace/       # ROS 2 side for Unity communication
│   │       └── unity_bridge/
│   │           └── package.xml
│   └── ch3_sensor_simulation/    # Chapter 3 code examples
│       ├── package.xml
│       ├── launch/
│       │   ├── lidar_2d.launch.py
│       │   ├── depth_camera.launch.py
│       │   └── imu_sensor.launch.py
│       ├── urdf/
│       │   └── sensor_platform.urdf  # Robot with all 3 sensors
│       ├── config/
│       │   ├── lidar_params.yaml
│       │   ├── camera_params.yaml
│       │   └── imu_params.yaml
│       └── validation/
│           ├── ground_truth_distances.py
│           └── sensor_noise_analysis.py
├── tests/
│   ├── ch1_physics/
│   │   ├── test_collision_detection.py
│   │   └── test_joint_constraints.py
│   ├── ch2_unity/
│   │   └── UnityProject/Tests/   # Unity Test Framework
│   └── ch3_sensors/
│       ├── test_lidar_output.py
│       └── test_depth_accuracy.py
├── docker/
│   ├── Dockerfile.gazebo         # ROS 2 Humble + Gazebo Harmonic
│   └── Dockerfile.unity          # Unity + ROS-TCP-Connector (headless)
├── .github/
│   └── workflows/
│       ├── gazebo-ci.yml         # Test Gazebo examples
│       └── unity-ci.yml          # Test Unity builds
└── README.md                     # Module 2 code examples overview
```

**Structure Decision**:

Educational content module requiring dual repositories:
1. **Docusaurus content**: Instructional text, diagrams, theory (in existing `physicalai-humanoidrobotics-book/`)
2. **Code examples**: Runnable ROS 2 + Gazebo + Unity projects (new `module-2-digital-twin/` repository)

This separation follows Module 1 pattern (`module-1-ros2-fundamentals/`) and keeps large binary assets (Unity projects, 3D meshes) out of the docs repository.

## Complexity Tracking

**No violations - this section is empty.**

All 10 constitution principles PASS without requiring justifications.

---

## Phase 0: Research & Technical Decisions

*To be filled by research.md generation - resolves any NEEDS CLARIFICATION items from Technical Context*

**Research Topics**:
1. Gazebo Harmonic vs Gazebo Classic: Version selection rationale for ROS 2 integration
2. Unity ROS-TCP-Connector vs Unity Robotics Hub: Best integration approach for 2025
3. Sensor plugin comparison: Gazebo vs Unity for LiDAR/Depth camera fidelity
4. URDF inertia tensor calculation: Tools and best practices for stable physics
5. Mermaid diagram patterns: Standardized templates for ROS 2 computation graphs and TF trees

**Expected Decisions** (to be validated in research.md):
- Gazebo Harmonic (newer, better ROS 2 support) over Gazebo Classic
- ROS-TCP-Connector (official Unity package) for ROS 2 integration
- Gazebo sensors for Tier A (more accurate physics), Unity sensors for visualization
- `inertial-calculator` tool or manual URDF validation with `check_urdf`
- Mermaid templates adapted from ROS 2 documentation and Module 1 diagrams

## Phase 1: Design Artifacts

*To be filled by data-model.md, contracts/, and quickstart.md generation*

**Expected Outputs**:

1. **data-model.md**: Learning content entities
   - Chapter entity (title, learning outcomes, prerequisites, exercises)
   - Code Example entity (type, file path, dependencies, validation)
   - Mermaid Diagram entity (type, description, accessibility text)
   - Exercise entity (instructions, acceptance criteria, solution hints)

2. **contracts/**:
   - `chapter-structure.md`: Standard MDX template for all 3 chapters
   - `code-example-format.md`: ROS 2 package structure, Unity scene conventions
   - `mermaid-diagram-spec.md`: Required diagram types, accessibility requirements

3. **quickstart.md**: Getting started guide
   - Prerequisites (Module 1 completion, hardware specs)
   - Installation (Gazebo Harmonic, Unity 2022 LTS, Docker setup)
   - Running first example (spawn robot in Gazebo, verify physics)
   - Troubleshooting common issues

## Next Steps

After `/sp.plan` completes:

1. **Review research.md**: Validate technical decisions for Gazebo/Unity integration
2. **Run `/sp.tasks`**: Generate implementation tasks from this plan
3. **Create content outline**: Use `contracts/chapter-structure.md` to draft MDX files
4. **Set up code repository**: Initialize `module-2-digital-twin/` with directory structure
5. **Implement Chapter 1**: Start with Gazebo physics (Priority P1) as foundation

**Estimated Timeline** (based on spec):
- Research + Design (this command): 2-3 hours
- Chapter 1 content + code: 12-16 hours
- Chapter 2 content + code: 16-20 hours (Unity setup overhead)
- Chapter 3 content + code: 12-16 hours
- Integration project + testing: 8-10 hours
- **Total**: 50-65 hours (2 weeks for content creator)

**Dependencies**:
- Module 1 must be published (students need URDF and ROS 2 fundamentals)
- Gazebo Harmonic installation documented (can reference Module 1 setup)
- Unity 2022 LTS availability (free personal license sufficient)
- ROS-TCP-Connector package stable (validate version in research phase)
