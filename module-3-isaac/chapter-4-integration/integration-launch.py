#!/usr/bin/env python3

"""
Full system integration launch file for Isaac ROS humanoid robot
Chapter 4: Integration and Advanced Topics

This launch file integrates Isaac Sim, Isaac ROS VSLAM, and Isaac ROS Nav2
into a complete humanoid robotics system.
"""

import os
import sys
import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart, OnProcessIO, OnExecutionComplete
from launch.events import Shutdown


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_vslam = LaunchConfiguration('enable_vslam', default='true')
    enable_nav2 = LaunchConfiguration('enable_nav2', default='true')
    enable_isaac_sim = LaunchConfiguration('enable_isaac_sim', default='true')
    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    enable_monitoring = LaunchConfiguration('enable_monitoring', default='true')
    robot_namespace = LaunchConfiguration('robot_namespace', default='humanoid_robot')
    params_file = LaunchConfiguration('params_file')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_enable_vslam_cmd = DeclareLaunchArgument(
        'enable_vslam',
        default_value='true',
        description='Enable Isaac ROS VSLAM'
    )

    declare_enable_nav2_cmd = DeclareLaunchArgument(
        'enable_nav2',
        default_value='true',
        description='Enable Isaac ROS Nav2'
    )

    declare_enable_isaac_sim_cmd = DeclareLaunchArgument(
        'enable_isaac_sim',
        default_value='true',
        description='Enable Isaac Sim integration'
    )

    declare_enable_rviz_cmd = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable RViz visualization'
    )

    declare_enable_monitoring_cmd = DeclareLaunchArgument(
        'enable_monitoring',
        default_value='true',
        description='Enable system monitoring'
    )

    declare_robot_namespace_cmd = DeclareLaunchArgument(
        'robot_namespace',
        default_value='humanoid_robot',
        description='Robot namespace for multi-robot deployments'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('isaac_ros_integration'),
            'config', 'integration_params.yaml'
        ]),
        description='Full path to the ROS2 parameters file'
    )

    # Group actions based on configuration
    isaac_sim_group = GroupAction(
        condition=IfCondition(enable_isaac_sim),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('isaac_ros_integration'),
                        'launch',
                        'isaac_sim.launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'robot_namespace': robot_namespace
                }.items()
            )
        ]
    )

    vslam_group = GroupAction(
        condition=IfCondition(enable_vslam),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('isaac_ros_integration'),
                        'launch',
                        'vslam.launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'robot_namespace': robot_namespace
                }.items()
            )
        ]
    )

    nav2_group = GroupAction(
        condition=IfCondition(enable_nav2),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('isaac_ros_integration'),
                        'launch',
                        'nav2.launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'robot_namespace': robot_namespace
                }.items()
            )
        ]
    )

    rviz_group = GroupAction(
        condition=IfCondition(enable_rviz),
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', PathJoinSubstitution([
                    FindPackageShare('isaac_ros_integration'),
                    'config', 'integration.rviz'
                ])],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    # Integration monitoring node
    integration_monitor_node = Node(
        package='isaac_ros_integration',
        executable='integration_monitor',
        name='integration_monitor',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_namespace': robot_namespace,
            'enable_monitoring': enable_monitoring
        }],
        condition=IfCondition(enable_monitoring),
        output='screen'
    )

    # System coordinator node
    system_coordinator_node = Node(
        package='isaac_ros_integration',
        executable='system_coordinator',
        name='system_coordinator',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_namespace': robot_namespace
        }],
        output='screen'
    )

    # Safety manager node
    safety_manager_node = Node(
        package='isaac_ros_integration',
        executable='safety_manager',
        name='safety_manager',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_namespace': robot_namespace
        }],
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_enable_vslam_cmd)
    ld.add_action(declare_enable_nav2_cmd)
    ld.add_action(declare_enable_isaac_sim_cmd)
    ld.add_action(declare_enable_rviz_cmd)
    ld.add_action(declare_enable_monitoring_cmd)
    ld.add_action(declare_robot_namespace_cmd)
    ld.add_action(declare_params_file_cmd)

    # Add groups
    ld.add_action(isaac_sim_group)
    ld.add_action(vslam_group)
    ld.add_action(nav2_group)
    ld.add_action(rviz_group)

    # Add individual nodes
    ld.add_action(integration_monitor_node)
    ld.add_action(system_coordinator_node)
    ld.add_action(safety_manager_node)

    return ld


if __name__ == '__main__':
    generate_launch_description()