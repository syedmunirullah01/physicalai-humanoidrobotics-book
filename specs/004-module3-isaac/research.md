# Research & Technical Decisions: Module 3 Isaac

**Date**: 2025-12-14
**Feature**: Module 3 - The AI-Robot Brain (NVIDIA Isaac™)
**Plan**: [plan.md](./plan.md)

## Overview

This document captures research findings and technical decisions for Module 3, addressing unknowns identified in the implementation plan. All decisions prioritize educational accessibility, simulation-first learning, and alignment with the Three-Tier Hardware Accessibility principle.

## R1: NVIDIA Isaac Sim Setup & Licensing

### Research Question
What are Isaac Sim installation requirements, licensing terms for educational use, and compatibility with ROS 2 Humble/Jazzy?

### Findings

**System Requirements**:
- **OS**: Ubuntu 20.04/22.04 LTS, Windows 10/11
- **GPU**: NVIDIA RTX series (RTX 2060 or higher), 8GB+ VRAM
- **CUDA**: 11.8+ (bundled with Omniverse)
- **RAM**: 32GB recommended (16GB minimum)
- **Disk**: 50GB+ free space for installation

**Licensing**:
- Isaac Sim is **free for individual use** via NVIDIA Omniverse
- Educational institutions can use under NVIDIA Developer Program
- No commercial license required for non-production learning environments
- Attribution recommended but not legally required

**ROS 2 Compatibility**:
- **Humble (ROS 2 LTS)**: Fully supported via ROS 2 Bridge
- **Jazzy**: Experimental support, requires building from source
- **Recommendation**: Target Humble as primary, provide Jazzy compatibility notes

### Decision

**✅ Use Isaac Sim 2023.1.1 as primary simulation platform**

**Rationale**:
- Free access removes cost barrier (Tier A accessibility)
- Photorealistic rendering superior to Gazebo for perception training
- Native USD format aligns with industry standards
- ROS 2 Bridge provides seamless integration

**Alternatives Considered**:
1. **Gazebo Harmonic**: Better ROS 2 integration, but lacks photorealism for perception
   - **Verdict**: Keep as fallback option for learners without NVIDIA GPUs
2. **Unity with ROS-TCP-Connector**: Good rendering, but limited robotics ecosystem
   - **Verdict**: Rejected; steeper learning curve for robotics-specific tasks

**Implementation Impact**:
- Chapter 1 focuses on Isaac Sim; Gazebo alternative provided in appendix
- Installation guide includes Omniverse setup walkthrough
- Sample scenes provided as downloadable USD files

---

## R2: Isaac ROS Packages & GPU Requirements

### Research Question
Which Isaac ROS packages are needed for VSLAM? What are minimum GPU requirements (VRAM, CUDA compute capability)?

### Findings

**Required Isaac ROS Packages** (Chapter 2):
- `isaac_ros_visual_slam`: Core VSLAM pipeline (cuVSLAM backend)
- `isaac_ros_image_pipeline`: GPU-accelerated image preprocessing
- `isaac_ros_nitros`: Zero-copy message passing for performance
- `isaac_ros_common`: Shared utilities and launch files

**Optional Packages**:
- `isaac_ros_nvblox`: 3D reconstruction (for advanced learners)
- `isaac_ros_apriltag`: Fiducial marker detection (for calibration exercises)

**GPU Tier Specifications**:

| Tier | Hardware | VSLAM Frame Rate | Use Case |
|------|----------|------------------|----------|
| **Minimum** | GTX 1660 Ti (6GB VRAM) | 15-20 Hz | Basic learning |
| **Recommended** | RTX 3060 (12GB VRAM) | 30+ Hz | Full real-time |
| **Optimal** | RTX 4070+ (12GB+ VRAM) | 60+ Hz | Advanced exercises |
| **Jetson** | Orin Nano (8GB) | 20-25 Hz | Tier B deployment |

**CUDA Compute Capability**: 7.5+ required (Turing architecture or newer)

### Decision

**✅ Target RTX 3060-class GPUs as baseline, provide Jetson Orin deployment guide**

**Rationale**:
- RTX 3060 widely available in gaming/workstation laptops (~$500-800 used market)
- 30 Hz VSLAM meets real-time requirements for learning scenarios
- Jetson Orin Nano ($499) aligns with Tier B budget constraint

**Alternatives Considered**:
1. **CPU-only SLAM (ORB-SLAM3)**: Lower barrier, but 3-5x slower
   - **Verdict**: Provide as fallback for learners without GPUs
2. **Cloud GPU instances**: Removes hardware barrier, but adds recurring costs
   - **Verdict**: Mentioned as option in setup guide, not primary path

**Implementation Impact**:
- Chapter 2 prerequisites explicitly state GPU requirements
- Performance benchmarks table helps learners set expectations
- CPU fallback instructions in troubleshooting appendix

---

## R3: Nav2 Bipedal Planning Capabilities

### Research Question
Does Nav2 natively support footstep planning for bipedal robots? What plugins/extensions are required?

### Findings

**Nav2 Native Capabilities**:
- Global planner: Supports arbitrary cost maps, suitable for high-level bipedal path planning
- Local planner (DWB): Designed for differential drive, **not** bipedal kinematics
- Recovery behaviors: Generic enough to adapt for bipedal use cases

**Gap Analysis**:
- ❌ No built-in footstep planner
- ❌ No Zero Moment Point (ZMP) stability checks
- ❌ No contact scheduling for bipedal gaits
- ✅ Plugin architecture allows custom local planners

**Existing Solutions**:
1. **Humanoid Path Planner (ROS 1 package)**: Not ported to ROS 2
2. **MoveIt 2 with custom planner**: Over-engineered for navigation curriculum
3. **Custom Nav2 plugin**: Requires C++ implementation

### Decision

**✅ Develop educational footstep planner as Nav2 plugin (Python + C++ optional)**

**Rationale**:
- Aligns with learning objective: "Design custom navigation strategies"
- Demonstrates Nav2 plugin architecture (valuable skill for learners)
- Simplified 2D footstep planning (not full 3D dynamics) appropriate for educational scope
- Python implementation accessible to target audience

**Scope Constraints**:
- **Included**: Discrete footstep selection on flat terrain, basic ZMP stability margin checks
- **Excluded**: Full dynamics simulation, compliance control, rough terrain traversal
- **Optional**: Integration with whole-body controller (advanced topic)

**Alternatives Considered**:
1. **Skip bipedal focus, use wheeled robot**: Easier but less differentiated curriculum
   - **Verdict**: Rejected; bipedal planning is unique selling point of Module 3
2. **Use pre-built MoveIt 2 planner**: Hides complexity, reduces learning value
   - **Verdict**: Rejected; black-box approach contradicts hands-on philosophy

**Implementation Impact**:
- Chapter 3 includes footstep planner implementation tutorial (70% provided, 30% learner exercise)
- Stability constraints explained with Mermaid diagrams
- Sample URDF with bipedal kinematics (e.g., Unitree G1 simplified model)

---

## R4: Sim-to-Real Transfer Techniques

### Research Question
What domain randomization parameters are most effective for perception model sim-to-real transfer?

### Findings

**Effective Randomization Parameters** (ranked by impact):
1. **Lighting** (Highest impact): Intensity, color temperature, shadow hardness
2. **Textures**: Object materials, floor patterns, wall colors
3. **Object Poses**: Position jitter, rotation variance
4. **Camera Parameters**: Exposure, gain, noise level
5. **Scene Clutter**: Number of objects, occlusion density

**Recommended Ranges** (based on NVIDIA research papers):
- Lighting intensity: ±40% of baseline
- Texture randomization: 5-10 material variants per object category
- Object pose jitter: ±10cm translation, ±15° rotation
- Camera noise: Gaussian noise σ = 0.01-0.03

**Dataset Size Requirements**:
- **Minimum**: 1,000 images for basic perception models (single object class)
- **Recommended**: 10,000+ images for robust multi-class detection
- **Validation**: 20% held-out real-world images to measure sim-to-real gap

### Decision

**✅ Implement tiered randomization strategy with Isaac Sim Replicator**

**Tier 1 (Beginner)**: Lighting + basic texture randomization (Chapter 1, Exercise 1)
**Tier 2 (Intermediate)**: Add object pose jitter (Chapter 1, Exercise 2)
**Tier 3 (Advanced)**: Full randomization including camera params (Chapter 1, optional)

**Rationale**:
- Progressive complexity matches Bloom's taxonomy (Analyze → Apply)
- Isaac Sim Replicator API simplifies implementation
- Tiered approach shows incremental impact on model performance

**Validation Metrics**:
- Precision/Recall on synthetic validation set
- Sim-to-real accuracy drop (measured on sample real dataset)
- Learners visualize distribution shift via t-SNE embeddings

**Implementation Impact**:
- Chapter 1, Section 5: Domain Randomization tutorial with code examples
- Pre-configured randomization profiles (YAML files) for common scenarios
- Success criteria SC-007 (>85% accuracy) validated with provided real-world test set

---

## R5: Learning Content Scope

### Research Question
What depth of mathematical detail (SLAM optimization, ZMP stability) is appropriate for target audience?

### Findings

**Target Audience** (from spec assumptions):
- Completed Module 1 (ROS 2 Fundamentals) and Module 2 (Digital Twin)
- Engineers/researchers with programming background
- Familiar with linear algebra, basic probability theory
- **Not** robotics PhDs; practical focus over theoretical depth

**Mathematical Prerequisites** (survey of similar curricula):
- **SLAM Theory**: Understanding of graph optimization > deriving Jacobians
- **ZMP Stability**: Concept of Center of Mass projection > full dynamics equations
- **Perception**: Intuition for feature matching > epipolar geometry proofs

### Decision

**✅ "Concept + Code" approach: Explain intuition with diagrams, demonstrate with runnable code**

**Content Structure** (per chapter):
1. **Conceptual Overview** (20%): High-level intuition, Mermaid diagrams, real-world analogies
2. **Hands-On Implementation** (60%): Guided code examples, parameter tuning, debugging
3. **Mathematical Deep-Dive** (20%): Optional expandable sections for rigorous learners

**Examples**:
- **SLAM**: Explain "pose graph" visually → Run cuVSLAM → Optional: Bundle Adjustment math
- **Footstep Planning**: Show stability polygon → Implement ZMP check → Optional: ZMP dynamics derivation

**Rationale**:
- Aligns with Bloom's "Apply" level (use tools effectively > derive from first principles)
- Accommodates diverse learners (practitioners skip optional sections, theorists expand)
- Faster time-to-value: learners run code in Chapter 1, not Chapter 3

**Implementation Impact**:
- Each chapter includes "Math Background" expandable section (Docusaurus `<details>`)
- References to external resources (papers, textbooks) for deep dives
- Code comments explain *why* parameters matter, not just *what* they are

---

## Technology Stack Summary

| Layer | Technology | Justification |
|-------|------------|---------------|
| **Simulation** | Isaac Sim 2023.1.1, Gazebo Harmonic (fallback) | Photorealism + free access |
| **ROS 2** | Humble (primary), Jazzy (notes) | LTS stability |
| **VSLAM** | Isaac ROS cuVSLAM, ORB-SLAM3 (CPU fallback) | GPU acceleration + accessibility |
| **Navigation** | Nav2 + custom footstep plugin | Standard framework + learning value |
| **Languages** | Python 3.10+ (primary), C++ (optional) | Accessibility + performance option |
| **Data Formats** | USD (scenes), ROS bag (sensor data), YAML (configs) | Industry standards |

---

## Architecture Decisions Summary

| ID | Decision | Trade-Off | Status |
|----|----------|-----------|--------|
| **AD-001** | Isaac Sim primary, Gazebo fallback | Dual maintenance vs max reach | ✅ Accepted |
| **AD-002** | Tier A simulation-only core | Hardware learners wait vs early value | ✅ Accepted |
| **AD-003** | Pre-generated sample datasets | Less hands-on vs lower compute barrier | ✅ Accepted |
| **AD-004** | Bipedal focus (not wheeled robots) | Higher complexity vs differentiation | ✅ Accepted |
| **AD-005** | Custom Nav2 footstep plugin | Implementation effort vs learning value | ✅ Accepted |

---

## Open Questions & Future Research

1. **Jetson Power Modes**: Which power profile (10W vs 25W) optimal for VSLAM frame rate?
   - **Action**: Benchmark in Chapter 2 Tier B implementation
2. **Cloud GPU Options**: Cost-benefit analysis of AWS/GCP GPU instances for Isaac Sim
   - **Action**: Add to quickstart guide as alternative to local GPU
3. **Perception Models**: Which pre-trained models best for humanoid navigation?
   - **Action**: Survey YOLOv8, RT-DETR for Chapter 2 optional exercises

---

## References

1. NVIDIA Isaac Sim Documentation: https://docs.omniverse.nvidia.com/isaacsim
2. Isaac ROS GitHub: https://github.com/NVIDIA-ISAAC-ROS
3. Nav2 Documentation: https://navigation.ros.org
4. Domain Randomization for Sim-to-Real Transfer (Tobin et al., 2017)
5. Humanoid Locomotion Planning (Kajita et al., Bipedal Robotics Book)
