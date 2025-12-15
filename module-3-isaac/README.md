# Module 3: The AI-Robot Brain (NVIDIA Isaac™)

**Focus**: Advanced perception and training for humanoid robotics using NVIDIA Isaac platform

## Overview

Module 3 provides comprehensive hands-on training in:
- **Chapter 1**: NVIDIA Isaac Sim - Photorealistic simulation and synthetic data generation
- **Chapter 2**: Isaac ROS - Hardware-accelerated Visual SLAM (VSLAM)
- **Chapter 3**: Nav2 - Bipedal path planning and autonomous navigation

This module builds upon Module 1 (ROS 2 Fundamentals) and Module 2 (Digital Twin Engineering) to deliver end-to-end AI-powered robotics capabilities.

## Prerequisites

### Hardware Requirements

**Tier A: Simulation Only** (Minimum)
- Ubuntu 22.04 LTS or Windows 11
- NVIDIA RTX 2060 or higher (6GB+ VRAM)
- 16GB RAM (32GB recommended)
- 100GB free disk space
- CUDA Compute Capability 7.5+

**Tier B: Edge AI Deployment** (Optional)
- NVIDIA Jetson Orin Nano (8GB) or Orin NX
- Expected VSLAM: 20-25 Hz at 848x480

**Tier C: Physical Robot** (Advanced)
- Humanoid robot with ROS 2 support
- Stereo camera setup

### Software Prerequisites

- **Required**: ROS 2 Humble (primary) or Jazzy (with compatibility notes)
- **Required**: NVIDIA Isaac Sim 2023.1.1+ (via Omniverse)
- **Required**: Python 3.10+
- **Recommended**: Ubuntu 22.04 LTS
- **GPU Drivers**: NVIDIA 535+ with CUDA 11.8+

### Knowledge Prerequisites

- Completed Module 1: ROS 2 Fundamentals
- Completed Module 2: Digital Twin Engineering
- Basic command line proficiency
- Python programming experience

## Quick Start

### 1. Install Dependencies

```bash
cd module-3-isaac
pip3 install -r requirements.txt
```

### 2. Install NVIDIA Isaac Sim

Follow the detailed setup guide in `../specs/004-module3-isaac/quickstart.md`

### 3. Build ROS 2 Workspace

```bash
# From repository root
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select module3_isaac_sim module3_vslam module3_navigation
source install/setup.bash
```

### 4. Verify Installation

```bash
# Run validation script
python3 shared/utils/tier_detection.py

# Expected output:
# Detected Tier: A (Simulation)
# GPU: NVIDIA RTX 3060 (12GB VRAM)
# CUDA: 11.8
# Status: Ready
```

## Module Structure

```
module-3-isaac/
├── README.md                    # This file
├── requirements.txt             # Python dependencies
├── package.xml                  # ROS 2 package metadata
├── chapter-1-isaac-sim/         # Chapter 1: Simulation & Synthetic Data
│   ├── 01-intro.mdx through 06-sim-to-real.mdx
│   ├── assets/                  # Sample scenes and configs
│   ├── config/                  # Randomization configurations
│   └── exercises/               # Hands-on Python exercises
├── chapter-2-isaac-ros/         # Chapter 2: Visual SLAM
│   ├── 01-intro.mdx through 07-debugging.mdx
│   ├── launch/                  # ROS 2 launch files
│   ├── config/                  # VSLAM parameters
│   └── exercises/               # VSLAM exercises
├── chapter-3-nav2/              # Chapter 3: Bipedal Navigation
│   ├── 01-intro.mdx through 07-integration.mdx
│   ├── launch/                  # Navigation launch files
│   ├── config/                  # Nav2 and footstep planner configs
│   ├── urdf/                    # Bipedal robot models
│   ├── src/                     # Custom footstep planner plugin
│   └── exercises/               # Navigation exercises
├── shared/                      # Shared resources
│   ├── utils/                   # Utilities (tier detection, metrics, validation)
│   ├── datasets/                # Pre-generated datasets and ROS bags
│   └── assets/                  # Shared simulation assets
├── tests/                       # Validation tests for all chapters
├── docs/                        # Additional documentation
├── scripts/                     # Utility scripts
└── examples/                    # Reference solutions
```

## Learning Path

### Chapter 1: Isaac Sim (3-5 hours)

**Learning Objectives**:
- Create photorealistic simulation environments
- Configure virtual sensors (RGB, depth, stereo, LiDAR)
- Generate 1000+ labeled synthetic images
- Implement domain randomization strategies
- Understand sim-to-real transfer techniques

**Success Criteria**:
- SC-001: Generate 1000+ labeled images within 1 hour
- SC-007: Achieve >85% perception model accuracy

### Chapter 2: Isaac ROS VSLAM (3-5 hours)

**Learning Objectives**:
- Set up hardware-accelerated visual odometry
- Build 3D maps with loop closure detection
- Save, load, and relocalize in mapped environments
- Tune VSLAM performance and debug failures
- Understand GPU acceleration benefits

**Success Criteria**:
- SC-002: Achieve >30 Hz VSLAM frame rate
- SC-003: Maintain <2% drift over 100-meter paths

### Chapter 3: Nav2 Bipedal Navigation (3-5 hours)

**Learning Objectives**:
- Configure global and local path planners
- Implement custom footstep planner with ZMP stability
- Handle dynamic obstacle avoidance
- Configure recovery behaviors
- Integrate VSLAM localization with navigation

**Success Criteria**:
- SC-004: Generate footstep plans within 5 seconds
- SC-008: Achieve >95% navigation success rate
- SC-009: Demonstrate >70% recovery success rate

## Usage Examples

### Chapter 1: Generate Synthetic Dataset

```bash
cd chapter-1-isaac-sim/exercises
python3 ex2-generate-dataset.py --num-images 1000 --output ../datasets/my-dataset

# Expected: 1000 RGB images + depth maps + labels.json in COCO format
```

### Chapter 2: Run VSLAM in Simulation

```bash
# Terminal 1: Launch Isaac Sim with stereo camera
ros2 launch chapter-2-isaac-ros/launch/vslam-sim.launch.py

# Terminal 2: Start cuVSLAM
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py

# Terminal 3: Visualize in RViz
ros2 run rviz2 rviz2 -d chapter-2-isaac-ros/config/vslam.rviz
```

### Chapter 3: Autonomous Bipedal Navigation

```bash
# Launch full navigation stack
ros2 launch chapter-3-nav2/launch/nav2-sim.launch.py

# Send navigation goal via Python
cd chapter-3-nav2/exercises
python3 ex3-autonomous-nav.py --goal-x 5.0 --goal-y 3.0
```

## Hardware Tier Detection

The module automatically detects your hardware tier:

- **Tier A (Simulation)**: All chapters work in Isaac Sim
- **Tier B (Jetson)**: Chapters 2 & 3 can deploy to Jetson Orin
- **Tier C (Hardware)**: Full stack on physical humanoid robots

Set manually if needed:
```bash
export ROBOT_TIER=simulation  # or jetson, or hardware
```

## Troubleshooting

### Isaac Sim won't launch
```bash
# Check NVIDIA drivers
nvidia-smi

# Check Vulkan support
vulkaninfo | grep deviceName

# Review logs
tail -f ~/.nvidia-omniverse/logs/Kit/Isaac-Sim/2023.1/kit_*.log
```

### VSLAM frame rate too low
```bash
# Reduce camera resolution in launch file
# Edit chapter-2-isaac-ros/launch/vslam-sim.launch.py
# Change: width=1280 → 848, height=720 → 480

# Verify GPU usage
watch -n 1 nvidia-smi
```

### Nav2 planner not generating paths
```bash
# Check costmap
ros2 topic echo /global_costmap/costmap --once

# Verify TF tree
ros2 run tf2_tools view_frames
```

See `docs/faq.md` for comprehensive troubleshooting guide.

## Success Criteria Validation

Run comprehensive validation:
```bash
./scripts/validate-all-chapters.sh

# This runs:
# - tests/test_chapter1_simulation.py (FR-001 to FR-006)
# - tests/test_chapter2_vslam.py (FR-007 to FR-013)
# - tests/test_chapter3_navigation.py (FR-014 to FR-020)
```

Expected output:
```
✅ Chapter 1: All 6 functional requirements passed
✅ Chapter 2: All 7 functional requirements passed
✅ Chapter 3: All 7 functional requirements passed
✅ SC-001: 1247 images/hour (target: 1000+)
✅ SC-002: 35.2 Hz VSLAM (target: >30 Hz)
✅ SC-003: 1.8% drift (target: <2%)
✅ SC-004: 3.2s planning (target: <5s)
✅ SC-008: 97% success rate (target: >95%)
✅ SC-009: 74% recovery rate (target: >70%)

Overall: PASS
```

## Additional Resources

- **Specification**: `../specs/004-module3-isaac/spec.md`
- **Implementation Plan**: `../specs/004-module3-isaac/plan.md`
- **Setup Guide**: `../specs/004-module3-isaac/quickstart.md`
- **Data Model**: `../specs/004-module3-isaac/data-model.md`
- **API Contracts**: `../specs/004-module3-isaac/contracts/`

## Support

- **Documentation**: Check `docs/` directory
- **FAQ**: `docs/faq.md`
- **Hardware Tiers Guide**: `docs/hardware-tiers.md`
- **Glossary**: `docs/glossary.md`
- **GitHub Issues**: [repository/issues]
- **Discord**: #module3-isaac channel

## License

Part of the Physical AI & Humanoid Robotics educational series.

## Acknowledgments

- NVIDIA Isaac Sim and Isaac ROS teams
- Nav2 maintainers and community
- ROS 2 Humble contributors
