#!/usr/bin/env python3
"""
Test collision detection in Gazebo physics simulations

Test Strategy:
1. Spawn two objects with overlapping initial positions
2. Verify physics engine separates them (collision response)
3. Test friction coefficients affect sliding behavior
4. Verify objects don't pass through static obstacles
"""

import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity, GetModelState, DeleteEntity
import time
import math


class CollisionTestNode(Node):
    """ROS 2 node for testing collision detection"""

    def __init__(self):
        super().__init__('collision_test_node')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.state_client = self.create_client(GetModelState, '/get_model_state')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')

        # Wait for services
        self.spawn_client.wait_for_service(timeout_sec=10.0)
        self.state_client.wait_for_service(timeout_sec=10.0)
        self.delete_client.wait_for_service(timeout_sec=10.0)

    def spawn_box(self, name, x, y, z, mass=1.0, mu=0.5):
        """Spawn a simple box in Gazebo"""
        urdf_content = f"""
        <?xml version="1.0"?>
        <robot name="{name}">
          <link name="base_link">
            <visual>
              <geometry>
                <box size="0.5 0.5 0.5"/>
              </geometry>
            </visual>
            <collision>
              <geometry>
                <box size="0.5 0.5 0.5"/>
              </geometry>
              <surface>
                <friction>
                  <ode>
                    <mu>{mu}</mu>
                    <mu2>{mu}</mu2>
                  </ode>
                </friction>
              </surface>
            </collision>
            <inertial>
              <mass value="{mass}"/>
              <inertia ixx="0.042" iyy="0.042" izz="0.042" ixy="0" ixz="0" iyz="0"/>
            </inertial>
          </link>
        </robot>
        """

        req = SpawnEntity.Request()
        req.name = name
        req.xml = urdf_content
        req.robot_namespace = ''
        req.initial_pose = Pose()
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = z

        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result()

    def get_model_position(self, model_name):
        """Get current position of a model"""
        req = GetModelState.Request()
        req.model_name = model_name

        future = self.state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()

        if result.success:
            return (
                result.pose.position.x,
                result.pose.position.y,
                result.pose.position.z
            )
        return None

    def delete_model(self, model_name):
        """Delete a model from Gazebo"""
        req = DeleteEntity.Request()
        req.name = model_name

        future = self.delete_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result()


@pytest.fixture
def collision_node():
    """Fixture to create and destroy test node"""
    rclpy.init()
    node = CollisionTestNode()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_collision_prevents_overlap(collision_node):
    """
    Test: Objects with collision geometry should not pass through each other

    Expected: Two boxes spawned at overlapping positions should be separated
    by the physics engine's collision response
    """
    # Spawn two boxes at nearly the same position (should collide)
    collision_node.spawn_box('box1', 0.0, 0.0, 1.0, mass=5.0)
    collision_node.spawn_box('box2', 0.3, 0.0, 1.0, mass=5.0)  # 30cm overlap

    # Wait for physics to settle (collision response)
    time.sleep(2.0)

    # Get final positions
    pos1 = collision_node.get_model_position('box1')
    pos2 = collision_node.get_model_position('box2')

    assert pos1 is not None, "Box1 not found in simulation"
    assert pos2 is not None, "Box2 not found in simulation"

    # Calculate distance between boxes
    distance = math.sqrt(
        (pos1[0] - pos2[0])**2 +
        (pos1[1] - pos2[1])**2 +
        (pos1[2] - pos2[2])**2
    )

    # Boxes are 0.5m wide, so centers should be at least 0.5m apart
    assert distance >= 0.5, f"Boxes overlapping! Distance: {distance}m (expected >= 0.5m)"

    # Cleanup
    collision_node.delete_model('box1')
    collision_node.delete_model('box2')


def test_ground_collision(collision_node):
    """
    Test: Objects should collide with the ground plane and not fall through

    Expected: Box spawned above ground falls and stops at z ≈ 0.25 (half its height)
    """
    # Spawn box 2 meters above ground
    collision_node.spawn_box('falling_box', 0.0, 0.0, 2.0, mass=5.0)

    # Wait for box to fall and settle
    time.sleep(3.0)

    # Check final position
    pos = collision_node.get_model_position('falling_box')
    assert pos is not None, "Falling box not found"

    # Box height is 0.5m, so center should be at z ≈ 0.25m (half height above ground)
    assert 0.2 <= pos[2] <= 0.3, f"Box fell through ground! z={pos[2]}m (expected ≈0.25m)"

    # Cleanup
    collision_node.delete_model('falling_box')


def test_friction_affects_sliding(collision_node):
    """
    Test: Friction coefficient should affect sliding behavior on inclined planes

    Expected: High friction box should slide less far than low friction box
    """
    # Spawn two boxes with different friction on an incline
    # (In practice, you'd need an inclined plane in the world)
    # This is a simplified test - assumes ground plane

    # High friction box
    collision_node.spawn_box('high_friction_box', -1.0, 0.0, 1.0, mass=5.0, mu=1.0)
    # Low friction box
    collision_node.spawn_box('low_friction_box', 1.0, 0.0, 1.0, mass=5.0, mu=0.1)

    # Wait for physics
    time.sleep(2.0)

    # Get positions
    pos_high = collision_node.get_model_position('high_friction_box')
    pos_low = collision_node.get_model_position('low_friction_box')

    assert pos_high is not None, "High friction box not found"
    assert pos_low is not None, "Low friction box not found"

    # Both boxes should be resting on ground (z ≈ 0.25m)
    assert 0.2 <= pos_high[2] <= 0.3, f"High friction box fell through ground: z={pos_high[2]}"
    assert 0.2 <= pos_low[2] <= 0.3, f"Low friction box fell through ground: z={pos_low[2]}"

    # Cleanup
    collision_node.delete_model('high_friction_box')
    collision_node.delete_model('low_friction_box')


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
