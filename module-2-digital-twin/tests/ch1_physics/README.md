# Chapter 1 Physics Tests

This directory contains validation tests for Chapter 1: Physics Simulation in Gazebo.

## Test Files

### `test_collision_detection.py`
Tests collision detection and physics response:
- **test_collision_prevents_overlap**: Verifies objects don't pass through each other
- **test_ground_collision**: Ensures objects collide with ground plane correctly
- **test_friction_affects_sliding**: Validates friction coefficients work

### `test_joint_constraints.py`
Tests joint behavior in articulated robots:
- **test_joint_limits_respected**: Verifies revolute joints respect position limits
- **test_joint_starts_at_zero**: Checks default joint initialization
- **test_gravity_affects_joints**: Validates gravity pulls unsupported links

## Prerequisites

1. **Install test dependencies**:
   ```bash
   pip install pytest rclpy gazebo-msgs sensor-msgs geometry-msgs
   ```

2. **Start Gazebo** (in separate terminal):
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 launch gazebo_ros gazebo.launch.py
   ```

## Running Tests

### Run all Chapter 1 tests:
```bash
cd module-2-digital-twin
pytest tests/ch1_physics/ -v
```

### Run specific test file:
```bash
pytest tests/ch1_physics/test_collision_detection.py -v
```

### Run single test function:
```bash
pytest tests/ch1_physics/test_collision_detection.py::test_ground_collision -v
```

## Test Environment

These tests assume:
- Gazebo Harmonic or Fortress is running
- ROS 2 Humble or Jazzy is sourced
- Gazebo ROS 2 integration packages are installed

### Docker Test Environment (Recommended):
```bash
docker run -it --rm --network host -e DISPLAY=$DISPLAY module2-gazebo:humble
# Inside container:
source /opt/ros/humble/setup.bash
ros2 launch gazebo_ros gazebo.launch.py &
pytest tests/ch1_physics/ -v
```

## Expected Output

All tests should pass with output like:
```
tests/ch1_physics/test_collision_detection.py::test_collision_prevents_overlap PASSED
tests/ch1_physics/test_collision_detection.py::test_ground_collision PASSED
tests/ch1_physics/test_collision_detection.py::test_friction_affects_sliding PASSED
tests/ch1_physics/test_joint_constraints.py::test_joint_limits_respected PASSED
tests/ch1_physics/test_joint_constraints.py::test_joint_starts_at_zero PASSED
tests/ch1_physics/test_joint_constraints.py::test_gravity_affects_joints PASSED

========================== 6 passed in 25.34s ==========================
```

## Troubleshooting

### Issue: "Service '/spawn_entity' not available"
**Cause**: Gazebo is not running or ROS 2 integration is not loaded

**Solution**:
```bash
# Terminal 1: Launch Gazebo with ROS 2 integration
ros2 launch gazebo_ros gazebo.launch.py

# Terminal 2: Verify services are available
ros2 service list | grep spawn
# Expected: /spawn_entity

# Terminal 3: Run tests
pytest tests/ch1_physics/ -v
```

### Issue: Tests timeout
**Cause**: Simulation is running slower than real-time

**Solution**: Increase timeout values in test code or reduce physics complexity

### Issue: "ModuleNotFoundError: No module named 'gazebo_msgs'"
**Solution**:
```bash
pip install gazebo-msgs  # or
sudo apt install ros-humble-gazebo-msgs
```

## Continuous Integration

These tests are designed to run in CI/CD pipelines:
```yaml
# .github/workflows/test-module2.yml
- name: Run Chapter 1 Physics Tests
  run: |
    source /opt/ros/humble/setup.bash
    ros2 launch gazebo_ros gazebo.launch.py &
    sleep 5  # Wait for Gazebo to start
    pytest tests/ch1_physics/ -v --junitxml=test-results.xml
```

## Test Coverage

Current coverage:
- ✅ Collision detection (3 tests)
- ✅ Joint constraints (3 tests)
- ⚠️ Inertia calculations (manual verification recommended)
- ⚠️ Friction models (basic test, advanced scenarios TBD)

## Contributing Tests

When adding new tests:
1. Follow naming convention: `test_<feature>.py`
2. Use pytest fixtures for node initialization
3. Clean up spawned models after each test
4. Add docstrings explaining test purpose and expected behavior
5. Update this README with test descriptions
