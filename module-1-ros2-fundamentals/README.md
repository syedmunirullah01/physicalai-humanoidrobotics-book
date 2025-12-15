# Module 1: ROS 2 Fundamentals - Code Examples

This repository contains all code examples, URDF models, and launch files for **Module 1: The Robotic Nervous System (ROS 2)** of the Physical AI & Humanoid Robotics textbook.

## Repository Structure

```
module-1-ros2-fundamentals/
├── src/
│   ├── textbook_templates/       # Reusable templates (Node, launch files, package.xml)
│   ├── ch1_workspace_demo/       # Chapter 1: Workspace structure demo
│   ├── ch2_talker_listener/      # Chapter 2: Publisher/Subscriber examples
│   ├── ch3_robot_control/        # Chapter 3: TurtleBot3 control and LIDAR
│   ├── ch4_service_examples/     # Chapter 4: Custom services (AddTwoInts)
│   └── ch5_urdf_models/          # Chapter 5: URDF files and visualization
├── launch/                        # ROS 2 launch files for examples
├── tests/                         # pytest unit tests and launch tests
├── docker/                        # Docker devcontainer (ROS 2 Humble)
├── .github/workflows/             # CI pipeline (ruff, colcon build, pytest)
└── README.md                      # This file
```

## Prerequisites

- **ROS 2 Humble** or **ROS 2 Jazzy** installed
- **Python 3.10+**
- **Gazebo Harmonic** (for simulation)
- **colcon** build tool

### Quick Setup (Ubuntu 22.04/24.04)

```bash
# Install ROS 2 Humble (if not already installed)
sudo apt update
sudo apt install ros-humble-desktop-full

# Install dependencies
sudo apt install python3-colcon-common-extensions python3-rosdep
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-turtlebot3 ros-humble-turtlebot3-simulations
sudo apt install ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher ros-humble-rviz2

# Clone this repository
git clone <repository-url> ~/ros2_ws/src/module-1-ros2-fundamentals
cd ~/ros2_ws

# Install package dependencies with rosdep
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

### Docker Setup (Alternative)

If you prefer Docker for a consistent environment:

```bash
# Build Docker image (ROS 2 Humble)
cd module-1-ros2-fundamentals
docker build -f docker/Dockerfile.humble -t ros2-module1:humble .

# Run container with GUI support (for RViz/Gazebo)
xhost +local:docker  # Allow Docker to access display (Linux only)
docker run -it --rm \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd):/workspace/src/module-1-ros2-fundamentals \
  ros2-module1:humble
```

## Running Examples

### Chapter 1: Workspace Demo

```bash
# No code to run - just explore the workspace structure
cd ~/ros2_ws
ls -R src/
```

### Chapter 2: Talker/Listener (Publisher/Subscriber)

```bash
# Terminal 1: Run minimal publisher
ros2 run ch2_talker_listener minimal_publisher.py

# Terminal 2: Run minimal subscriber
ros2 run ch2_talker_listener minimal_subscriber.py

# Terminal 3: Monitor topic with CLI
ros2 topic echo /chatter
```

### Chapter 3: TurtleBot3 Control

```bash
# Terminal 1: Launch TurtleBot3 in Gazebo
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Run teleoperation node
ros2 run ch3_robot_control turtlebot3_teleop.py

# Terminal 3: Run obstacle detection
ros2 run ch3_robot_control lidar_obstacle_detector.py
```

### Chapter 4: Services (AddTwoInts)

```bash
# Terminal 1: Run service server
ros2 run ch4_service_examples add_two_ints_server.py

# Terminal 2: Call service from CLI
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 7}"

# Terminal 3: Run async service client
ros2 run ch4_service_examples add_two_ints_client_async.py
```

### Chapter 5: URDF Visualization

```bash
# Launch RViz with humanoid arm URDF
ros2 launch ch5_urdf_models display.launch.py

# Launch Gazebo with humanoid arm
ros2 launch ch5_urdf_models gazebo_arm.launch.py

# Validate URDF manually
check_urdf src/ch5_urdf_models/urdf/humanoid_arm_3dof.urdf
```

## Content Validation

### Readability Validation (For Textbook Content)

All textbook chapters MUST meet these readability targets:
- **Flesch-Kincaid Grade Level**: 13-15 (university undergraduate)
- **Flesch Reading Ease**: 30-50 (college-level difficulty)

To validate chapter content (from main repository):

```bash
# Install textstat
pip install textstat

# Run validation script (from E:\AIDD-HACKATHON)
python scripts/validate-readability.py physicalai-humanoidrobotics-book/docs/module1/chapter1-installation.mdx

# Expected output:
# Flesch-Kincaid Grade: 14.2 ✅
# Flesch Reading Ease: 42.5 ✅
# Word Count: 1,847
# Status: PASSED
```

If a chapter fails readability checks:
1. Simplify complex sentences (break long sentences)
2. Reduce passive voice
3. Replace jargon with plain language (or define technical terms in glossary)
4. Add examples and concrete explanations

### Code Quality Validation

```bash
# Run ruff linter
ruff check src/

# Run ruff formatter
ruff format src/

# Run pytest unit tests
pytest tests/ -v

# Run ROS 2 launch tests
colcon test --packages-select-regex "ch[1-5]_.*"
colcon test-result --all --verbose
```

## CI/CD Pipeline

This repository uses GitHub Actions for continuous integration:

### Automated Checks on Every Commit
1. **Linting**: `ruff check` and `ruff format`
2. **Build**: `colcon build` on ROS 2 Humble
3. **Tests**: `pytest` + `colcon test` for launch tests
4. **Code Quality**: `flake8`, `pep257`, `bandit` security scan

See `.github/workflows/ros2-ci.yml` for full configuration.

### Running CI Locally (Before Pushing)

```bash
# Lint code
ruff check src/ --fix

# Format code
ruff format src/

# Build and test
colcon build --symlink-install
colcon test
colcon test-result --all
```

## Template Usage

### Creating a New ROS 2 Package

```bash
# Copy package.xml template
cp src/textbook_templates/package.xml.template src/my_new_package/package.xml

# Edit package.xml and replace PACKAGE_NAME
sed -i 's/PACKAGE_NAME/my_new_package/g' src/my_new_package/package.xml

# Add dependencies (uncomment lines in package.xml as needed)
```

### Creating a New Python Node

```bash
# Copy minimal node template
cp src/textbook_templates/minimal_node_template.py src/my_new_package/my_node.py

# Edit my_node.py:
# 1. Rename class MinimalNodeTemplate → MyNode
# 2. Change node name in __init__: super().__init__('my_node')
# 3. Update topic names and message types
# 4. Implement your logic in timer_callback() and topic_callback()
```

### Creating a Launch File

```bash
# Copy launch template
cp launch/template.launch.py launch/my_launch.launch.py

# Edit my_launch.launch.py:
# 1. Replace 'your_package_name' with actual package name
# 2. Update URDF file path
# 3. Add your custom nodes
# 4. Configure RViz and Gazebo as needed
```

## Troubleshooting

### Common Issues

**Issue**: `colcon build` fails with "package not found"
- **Solution**: Run `rosdep install --from-paths src --ignore-src -r -y` to install missing dependencies

**Issue**: Gazebo doesn't launch
- **Solution**: Check if Gazebo is installed: `sudo apt install ros-humble-gazebo-ros-pkgs`

**Issue**: RViz shows "No transform from [base_link] to [world]"
- **Solution**: Verify `robot_state_publisher` is running and URDF is valid with `check_urdf`

**Issue**: Python node crashes immediately
- **Solution**: Check for exceptions in `__init__` method. Add `try-except` blocks and log errors.

### Getting Help

- **Documentation**: [ROS 2 Humble Docs](https://docs.ros.org/en/humble/)
- **Community**: [ROS Discourse](https://discourse.ros.org/)
- **Textbook**: See main repository for full module content

## Contributing

If you find bugs in code examples or have improvements:
1. Fork this repository
2. Create a feature branch: `git checkout -b fix/chapter3-typo`
3. Make changes and test locally: `colcon build && colcon test`
4. Ensure CI passes (ruff, pytest, colcon test)
5. Submit a pull request with clear description

## License

Apache 2.0 - See LICENSE file for details.

## Acknowledgments

- Built for the **Physical AI & Humanoid Robotics** textbook
- Uses ROS 2 Humble (Long-Term Support release)
- Tested with Gazebo Harmonic and TurtleBot3 simulation

**Happy Learning! 🤖**
