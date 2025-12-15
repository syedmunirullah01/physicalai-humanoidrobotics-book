# Contract: Code Example Format

**Purpose**: Standardized structure for ROS 2 and Unity code examples in Module 2
**Applies to**: All code in `module-2-digital-twin/` repository
**Date**: 2025-12-12

---

## ROS 2 Package Structure

### Standard Package Layout

```text
module-2-digital-twin/src/[PACKAGE_NAME]/
├── package.xml              # ROS 2 package manifest
├── setup.py                 # Python package setup (if Python nodes exist)
├── CMakeLists.txt           # Build configuration (if C++ nodes exist)
├── launch/                  # ROS 2 launch files
│   ├── [example_name].launch.py
│   └── README.md            # Launch file descriptions
├── urdf/                    # Robot models
│   ├── [robot_name].urdf
│   └── README.md            # URDF descriptions
├── worlds/                  # Gazebo world files (Chapter 1 only)
│   ├── [world_name].sdf
│   └── README.md
├── config/                  # YAML configuration files (Chapter 3)
│   ├── [sensor_name]_params.yaml
│   └── README.md
├── [PACKAGE_NAME]/          # Python source (if Python package)
│   ├── __init__.py
│   └── [node_script].py
└── README.md                # Package overview
```

### package.xml Template

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>[PACKAGE_NAME]</name>
  <version>1.0.0</version>
  <description>Module 2 code examples for [Chapter Title]</description>

  <maintainer email="textbook@example.com">Physical AI Textbook Team</maintainer>
  <license>Apache-2.0</license>

  <!-- Build tool dependencies -->
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- ROS 2 dependencies (adjust per example) -->
  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>

  <!-- Gazebo dependencies (Chapter 1 & 3) -->
  <depend>gazebo_ros</depend>
  <depend>gazebo_ros_pkgs</depend>
  <depend>robot_state_publisher</depend>

  <!-- Testing dependencies -->
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <test_depend>pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Launch File Template

```python
"""
Launch file for [EXAMPLE_NAME]

Usage:
    ros2 launch [PACKAGE_NAME] [FILENAME].launch.py

Expected behavior:
    [Describe what should happen when this launch file runs]
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world', default='empty_world')

    # Package paths
    pkg_share = FindPackageShare('[PACKAGE_NAME]')
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', '[ROBOT_NAME].urdf'])
    world_file = PathJoinSubstitution([pkg_share, 'worlds', '[WORLD_NAME].sdf'])

    # Gazebo launch (Chapter 1 & 3)
    gazebo_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('gazebo_ros'),
            'launch',
            'gazebo.launch.py'
        ]),
        launch_arguments={'world': world_file}.items()
    )

    # Robot state publisher (for URDF → TF transforms)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf_file]
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', '[ROBOT_NAME]',
            '-file', urdf_file,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        DeclareLaunchArgument('world', default_value='empty_world',
                              description='Gazebo world name'),
        gazebo_launch,
        robot_state_publisher,
        spawn_entity,
    ])
```

### URDF File Template (with Physics)

```xml
<?xml version="1.0"?>
<robot name="[ROBOT_NAME]">
  <!-- Base link (main body) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>

    <inertial>
      <!-- Mass in kg -->
      <mass value="1.0"/>
      <!-- Inertia tensor for box: ixx = (1/12) * m * (h^2 + d^2) -->
      <inertia
        ixx="0.0108" ixy="0.0" ixz="0.0"
        iyy="0.0242" iyz="0.0"
        izz="0.0292"/>
    </inertial>
  </link>

  <!-- Gazebo-specific physics properties -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.8</mu1>  <!-- Friction coefficient 1 -->
    <mu2>0.8</mu2>  <!-- Friction coefficient 2 -->
  </gazebo>

  <!-- [Add joints and additional links as needed] -->
</robot>
```

---

## Unity Project Structure

### Standard Unity Project Layout

```text
module-2-digital-twin/src/ch2_unity_rendering/UnityProject/
├── Assets/
│   ├── Scenes/
│   │   ├── [SceneName].unity          # Unity scene file
│   │   └── README.md                   # Scene descriptions
│   ├── Scripts/
│   │   ├── ROSConnection.cs           # ROS-TCP-Connector setup
│   │   ├── [CustomScript].cs          # Scene-specific logic
│   │   └── README.md                   # Script descriptions
│   ├── Prefabs/
│   │   ├── [RobotName].prefab         # Reusable robot model
│   │   └── README.md
│   ├── Materials/
│   │   ├── [MaterialName].mat         # PBR materials
│   │   └── README.md
│   └── Models/
│       ├── [MeshName].fbx             # Imported 3D models
│       └── README.md
├── Packages/
│   └── manifest.json                   # Unity package dependencies
├── ProjectSettings/
│   └── [Unity generated files]
└── README.md                           # Unity project overview
```

### C# Script Template (ROS Integration)

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

/// <summary>
/// [SCRIPT PURPOSE - e.g., "Controls robot joint positions via ROS 2 topics"]
///
/// ROS 2 Topics:
/// - Subscribes to: /joint_commands (geometry_msgs/Twist)
/// - Publishes to: /robot_state (sensor_msgs/JointState)
///
/// Usage: Attach this script to the robot GameObject in the Unity scene.
/// </summary>
public class [ScriptName] : MonoBehaviour
{
    [Header("ROS Settings")]
    [Tooltip("Name of the ROS topic to subscribe to")]
    public string topicName = "/joint_commands";

    [Header("Robot References")]
    [Tooltip("Assign robot joint GameObjects in the Inspector")]
    public Transform[] joints;

    private ROSConnection ros;

    void Start()
    {
        // Initialize ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<MVector3Msg>(topicName, OnJointCommandReceived);

        Debug.Log($"[{GetType().Name}] Subscribed to {topicName}");
    }

    /// <summary>
    /// Callback for ROS messages
    /// </summary>
    private void OnJointCommandReceived(MVector3Msg message)
    {
        // Example: Apply received data to robot joints
        if (joints.Length > 0)
        {
            joints[0].localRotation = Quaternion.Euler(
                message.x * Mathf.Rad2Deg,
                message.y * Mathf.Rad2Deg,
                message.z * Mathf.Rad2Deg
            );
        }
    }

    void Update()
    {
        // Publish robot state back to ROS (optional)
        // PublishRobotState();
    }

    void OnDestroy()
    {
        // Cleanup ROS subscriptions
        if (ros != null)
        {
            ros.Unsubscribe(topicName);
        }
    }
}
```

### Unity Scene Setup Checklist

- [ ] **ROS TCP Connector** GameObject added to scene
- [ ] **ROS Settings** configured (IP address, port 10000)
- [ ] **Lighting** configured (Directional Light + Ambient Occlusion)
- [ ] **Post-Processing** stack added (optional - for photorealism)
- [ ] **Robot Prefab** instantiated with articulation bodies
- [ ] **Ground Plane** added with physics collider
- [ ] **Camera** positioned for clear view of robot
- [ ] **UI Canvas** (optional) for debug info

---

## Testing Requirements

### ROS 2 Package Tests

**Location**: `module-2-digital-twin/tests/[CHAPTER_NAME]/`

**Test Template** (pytest):

```python
"""
Test suite for [EXAMPLE_NAME]

Validates that the code example:
1. Launches without errors
2. Publishes expected ROS 2 topics
3. Produces correct physics behavior (Chapter 1)
4. Sensors publish valid data (Chapter 3)
"""

import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time


class TestExampleName(Node):
    def __init__(self):
        super().__init__('test_[example_name]')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10
        )
        self.received_message = None

    def listener_callback(self, msg):
        self.received_message = msg


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_topic_publishes(ros_context):
    """Verify that expected topic is being published"""
    test_node = TestExampleName()

    # Spin for a few seconds to receive messages
    timeout = time.time() + 5.0
    while time.time() < timeout and test_node.received_message is None:
        rclpy.spin_once(test_node, timeout_sec=0.1)

    assert test_node.received_message is not None, \
        "No message received on /scan topic within timeout"

    test_node.destroy_node()


def test_sensor_data_format(ros_context):
    """Verify sensor data format matches specification"""
    test_node = TestExampleName()

    # Wait for message
    timeout = time.time() + 5.0
    while time.time() < timeout and test_node.received_message is None:
        rclpy.spin_once(test_node, timeout_sec=0.1)

    msg = test_node.received_message
    assert len(msg.ranges) > 0, "LaserScan ranges array is empty"
    assert msg.range_min < msg.range_max, \
        f"Invalid range: min={msg.range_min}, max={msg.range_max}"

    test_node.destroy_node()
```

### Unity Tests

**Location**: `UnityProject/Assets/Tests/`

**Test Template** (Unity Test Framework):

```csharp
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using System.Collections;

public class ROSConnectionTests
{
    [UnityTest]
    public IEnumerator RobotLoadsInScene()
    {
        // Load the HRI scene
        var scene = UnityEngine.SceneManagement.SceneManager.LoadSceneAsync("HumanoidHRI");
        yield return scene;

        // Find robot GameObject
        var robot = GameObject.Find("Humanoid");
        Assert.IsNotNull(robot, "Robot GameObject not found in scene");
    }

    [Test]
    public void ROSConnectionConfigured()
    {
        var rosConnection = GameObject.FindObjectOfType<Unity.Robotics.ROSTCPConnector.ROSConnection>();
        Assert.IsNotNull(rosConnection, "ROSConnection not found in scene");
        Assert.AreEqual(10000, rosConnection.rosPort, "ROS port should be 10000");
    }
}
```

---

## Documentation Requirements

### README.md (per package)

```markdown
# [Package Name]

**Module**: Module 2 - The Digital Twin
**Chapter**: [Chapter Number and Title]
**Description**: [One-paragraph overview]

## Contents

- `launch/`: [List launch files with brief descriptions]
- `urdf/`: [List URDF models]
- `config/`: [List configuration files - Chapter 3 only]

## Quick Start

### Prerequisites

- ROS 2 Humble or Jazzy
- Gazebo Harmonic [if applicable]
- [Additional dependencies...]

### Installation

```bash
cd ~/ros2_ws/src
git clone [REPO_URL]
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select [PACKAGE_NAME]
source install/setup.bash
```

### Run Example

```bash
ros2 launch [PACKAGE_NAME] [LAUNCH_FILE].launch.py
```

**Expected Output**: [Describe what should happen]

## Troubleshooting

[Common issues and solutions]

## License

Apache 2.0 - See LICENSE file in repository root
```

---

## Quality Checklist

Before submitting code examples:

**ROS 2 Code**:
- [ ] `package.xml` complete with all dependencies
- [ ] `README.md` in package root with quickstart
- [ ] Launch files have docstrings
- [ ] URDF models validated with `check_urdf`
- [ ] Tests pass: `colcon test --packages-select [PACKAGE_NAME]`
- [ ] Code follows rclpy patterns from Module 1

**Unity Code**:
- [ ] C# scripts have XML doc comments
- [ ] Scene includes ROS TCP Connector GameObject
- [ ] Prefabs are reusable across scenes
- [ ] Builds without errors (File → Build Settings → Build)
- [ ] Tests pass in Unity Test Runner

**Safety**:
- [ ] Motor control code has E-Stop documentation
- [ ] Force application examples warn about physics instability
- [ ] No hardcoded dangerous values (excessive velocities, torques)

---

**Status**: Code format contracts complete. Apply to all Module 2 examples.
