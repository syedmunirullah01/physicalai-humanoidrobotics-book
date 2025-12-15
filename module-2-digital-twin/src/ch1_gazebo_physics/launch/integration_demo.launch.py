#!/usr/bin/env python3
"""
Integration Demo Launch File

Launches complete digital twin system:
- Gazebo with physics and sensors
- ROS 2 bridge for Unity
- RViz2 for debugging

Usage:
    ros2 launch ch1_gazebo_physics integration_demo.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_ch1 = get_package_share_directory('ch1_gazebo_physics')
    pkg_ch2 = get_package_share_directory('unity_bridge')

    # URDF file
    urdf_file = os.path.join(pkg_ch1, 'urdf', 'integration_humanoid.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # 1. Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': 'warehouse.world'}.items()
    )

    # 2. Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'humanoid_robot',
            '-file', urdf_file,
            '-x', '0.0', '-y', '0.0', '-z', '0.5'
        ],
        output='screen'
    )

    # 3. Robot State Publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    # 4. Unity ROS Bridge (Chapter 2)
    unity_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ch2, 'launch', 'unity_ros_bridge.launch.py')
        )
    )

    # 5. RViz2 for debugging (delayed start)
    rviz_config = os.path.join(pkg_ch1, 'rviz', 'integration.rviz')
    rviz = TimerAction(
        period=5.0,  # Wait 5 seconds for Gazebo to stabilize
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        robot_state_pub,
        unity_bridge,
        rviz
    ])
