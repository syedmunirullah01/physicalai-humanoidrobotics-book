#!/usr/bin/env python3
"""
Launch file for spawning 3dof_arm.urdf in Gazebo

Learning objectives:
- Spawning articulated robots with multiple revolute joints
- Understanding joint limits and dynamics
- Observing gravity effects on multi-link systems
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for 3-DOF articulated arm example"""

    # Get package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_ch1 = get_package_share_directory('ch1_gazebo_physics')

    # Path to URDF file
    urdf_file = os.path.join(pkg_ch1, 'urdf', '3dof_arm.urdf')

    # Read URDF file content
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Gazebo launch (starts empty world)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'verbose': 'true'}.items()
    )

    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', '3dof_arm',
            '-file', urdf_file,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'  # Spawn base slightly above ground
        ],
        output='screen'
    )

    # Robot State Publisher (publishes TF transforms)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Joint State Publisher GUI (for manual joint control in RViz2)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
        robot_state_publisher,
        joint_state_publisher_gui
    ])
