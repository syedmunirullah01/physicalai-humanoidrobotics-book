#!/usr/bin/env python3
"""
Test joint constraints in articulated robots

Test Strategy:
1. Spawn robot with revolute joints and joint limits
2. Apply torques beyond limits and verify joints stop at limits
3. Test continuous joints rotate freely without limits
4. Verify joint dynamics (damping, friction) affect motion
"""

import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import SpawnEntity, ApplyJointEffort, DeleteEntity
import time
import math


class JointTestNode(Node):
    """ROS 2 node for testing joint constraints"""

    def __init__(self):
        super().__init__('joint_test_node')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.effort_client = self.create_client(ApplyJointEffort, '/apply_joint_effort')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')

        self.joint_states = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Wait for services
        self.spawn_client.wait_for_service(timeout_sec=10.0)
        self.effort_client.wait_for_service(timeout_sec=10.0)
        self.delete_client.wait_for_service(timeout_sec=10.0)

    def joint_state_callback(self, msg):
        """Store latest joint states"""
        self.joint_states = msg

    def spawn_simple_arm(self, name):
        """Spawn a simple 2-link arm with joint limits"""
        urdf_content = f"""
        <?xml version="1.0"?>
        <robot name="{name}">
          <link name="base_link">
            <visual>
              <geometry>
                <cylinder radius="0.1" length="0.2"/>
              </geometry>
            </visual>
            <collision>
              <geometry>
                <cylinder radius="0.1" length="0.2"/>
              </geometry>
            </collision>
            <inertial>
              <mass value="2.0"/>
              <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
            </inertial>
          </link>

          <link name="link1">
            <visual>
              <origin xyz="0 0 0.25" rpy="0 0 0"/>
              <geometry>
                <box size="0.1 0.1 0.5"/>
              </geometry>
            </visual>
            <collision>
              <origin xyz="0 0 0.25" rpy="0 0 0"/>
              <geometry>
                <box size="0.1 0.1 0.5"/>
              </geometry>
            </collision>
            <inertial>
              <origin xyz="0 0 0.25" rpy="0 0 0"/>
              <mass value="1.0"/>
              <inertia ixx="0.022" iyy="0.022" izz="0.002" ixy="0" ixz="0" iyz="0"/>
            </inertial>
          </link>

          <joint name="joint1" type="revolute">
            <parent link="base_link"/>
            <child link="link1"/>
            <origin xyz="0 0 0.1" rpy="0 0 0"/>
            <axis xyz="0 1 0"/>
            <limit lower="-1.57" upper="1.57" effort="10.0" velocity="2.0"/>
            <dynamics damping="0.5" friction="0.1"/>
          </joint>
        </robot>
        """

        req = SpawnEntity.Request()
        req.name = name
        req.xml = urdf_content
        req.robot_namespace = ''

        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result()

    def apply_torque(self, joint_name, effort, duration):
        """Apply torque to a joint"""
        req = ApplyJointEffort.Request()
        req.joint_name = joint_name
        req.effort = effort
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()

        future = self.effort_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result()

    def get_joint_position(self, joint_name):
        """Get current position of a joint"""
        if self.joint_states is None:
            return None

        try:
            idx = self.joint_states.name.index(joint_name)
            return self.joint_states.position[idx]
        except (ValueError, IndexError):
            return None

    def delete_model(self, model_name):
        """Delete a model from Gazebo"""
        req = DeleteEntity.Request()
        req.name = model_name

        future = self.delete_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result()


@pytest.fixture
def joint_node():
    """Fixture to create and destroy test node"""
    rclpy.init()
    node = JointTestNode()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_joint_limits_respected(joint_node):
    """
    Test: Revolute joints should respect position limits

    Expected: Joint with limits [-1.57, 1.57] should stop at these bounds
    even when excessive torque is applied
    """
    # Spawn arm with joint limits
    joint_node.spawn_simple_arm('test_arm')

    # Wait for spawn
    time.sleep(1.0)

    # Apply large positive torque (try to exceed upper limit)
    joint_node.apply_torque('joint1', 50.0, 2.0)
    time.sleep(2.5)

    # Check joint position
    pos = joint_node.get_joint_position('joint1')
    assert pos is not None, "Joint state not received"

    # Joint should be at or below upper limit (1.57 rad ≈ 90°)
    assert pos <= 1.57 + 0.1, f"Joint exceeded upper limit: {pos}rad (max: 1.57rad)"

    # Apply large negative torque (try to exceed lower limit)
    joint_node.apply_torque('joint1', -50.0, 2.0)
    time.sleep(2.5)

    pos = joint_node.get_joint_position('joint1')
    assert pos is not None, "Joint state not received"

    # Joint should be at or above lower limit (-1.57 rad ≈ -90°)
    assert pos >= -1.57 - 0.1, f"Joint exceeded lower limit: {pos}rad (min: -1.57rad)"

    # Cleanup
    joint_node.delete_model('test_arm')


def test_joint_starts_at_zero(joint_node):
    """
    Test: Joints should initialize at zero position by default

    Expected: Newly spawned joints without initial_position should be at 0.0 rad
    """
    joint_node.spawn_simple_arm('zero_test_arm')
    time.sleep(1.0)

    pos = joint_node.get_joint_position('joint1')
    assert pos is not None, "Joint state not received"

    # Joint should be near zero (within 0.01 rad tolerance)
    assert abs(pos) < 0.01, f"Joint not at zero: {pos}rad (expected ≈0.0rad)"

    # Cleanup
    joint_node.delete_model('zero_test_arm')


def test_gravity_affects_joints(joint_node):
    """
    Test: Gravity should cause unsupported links to fall

    Expected: Arm link should droop downward under gravity when no torque is applied
    """
    joint_node.spawn_simple_arm('gravity_test_arm')
    time.sleep(1.0)

    # Initial position (should be near zero)
    pos_initial = joint_node.get_joint_position('joint1')
    assert pos_initial is not None, "Joint state not received"

    # Wait for gravity to pull link down
    time.sleep(3.0)

    pos_final = joint_node.get_joint_position('joint1')
    assert pos_final is not None, "Joint state not received"

    # Joint should have moved downward (negative angle) due to gravity
    # Allow small tolerance for damping effects
    assert pos_final < pos_initial - 0.1, \
        f"Joint did not fall under gravity: initial={pos_initial}rad, final={pos_final}rad"

    # Cleanup
    joint_node.delete_model('gravity_test_arm')


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
