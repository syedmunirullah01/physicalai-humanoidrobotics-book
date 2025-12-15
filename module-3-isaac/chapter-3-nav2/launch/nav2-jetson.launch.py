#!/usr/bin/env python3

"""
Launch file for Isaac ROS Nav2 on Jetson Orin platforms
Chapter 3: Isaac ROS Nav2 - Jetson Deployment
"""

import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    jetson_power_mode = LaunchConfiguration('jetson_power_mode')

    # Launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('isaac_ros_nav2'),
            'config', 'nav2_params_jetson.yaml'
        ]),
        description='Full path to the ROS2 parameters file optimized for Jetson'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',  # Use composition for Jetson memory efficiency
        description='Whether to use composed bringup'
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='True',
        description='Whether to respawn if a node crashes'
    )

    declare_jetson_power_mode_cmd = DeclareLaunchArgument(
        'jetson_power_mode',
        default_value='MODE_15W',
        description='Jetson power mode: MODE_15W, MODE_MAXN, etc.'
    )

    # Set environment variables for Jetson optimization
    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower']

    # Remappings optimized for Jetson
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('/cmd_vel', 'cmd_vel'),
                  ('/odom', '/vslam/odometry'),  # Use VSLAM odometry from Isaac ROS
                  ('/initialpose', 'initialpose'),
                  ('/goal_pose', 'goal_pose'),
                  ('/map', 'map'),
                  ('/scan', 'scan')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    # Navigation server (composed for Jetson memory efficiency)
    nav2_container = Node(
        condition=IfCondition(use_composition),
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[configured_params, {'autostart': autostart}],
        remappings=remappings,
        output='screen',
        # Jetson-specific environment variables
        additional_env={
            'CUDA_DEVICE_ORDER': 'PCI_BUS_ID',
            'CUDA_VISIBLE_DEVICES': '0',
            'PYTHONUNBUFFERED': '1',
            'OMP_NUM_THREADS': '4',  # Limit OpenMP threads on Jetson
            'RCUTILS_LOGGING_BUFFERED_STREAM': '1'
        }
    )

    # AMCL node (optimized for Jetson)
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[configured_params],
        remappings=remappings,
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        # Jetson-specific environment variables
        additional_env={
            'CUDA_DEVICE_ORDER': 'PCI_BUS_ID',
            'CUDA_VISIBLE_DEVICES': '0',
            'RCUTILS_LOGGING_BUFFERED_STREAM': '1'
        }
    )

    # Planner Server (optimized for Jetson)
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[configured_params],
        remappings=remappings,
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0
    )

    # Controller Server (optimized for Jetson)
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[configured_params],
        remappings=remappings,
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0
    )

    # Behavior Server (optimized for Jetson)
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[configured_params],
        remappings=remappings,
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0
    )

    # BT Navigator (optimized for Jetson)
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[configured_params],
        remappings=remappings,
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0
    )

    # Waypoint Follower (optimized for Jetson)
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        parameters=[configured_params],
        remappings=remappings,
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0
    )

    # Lifecycle Manager (optimized for Jetson)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                   {'autostart': autostart},
                   {'node_names': lifecycle_nodes}],
        # Jetson-specific environment variables
        additional_env={
            'RCUTILS_LOGGING_BUFFERED_STREAM': '1'
        }
    )

    # Isaac ROS VSLAM integration node (Jetson optimized)
    isaac_ros_vslam_integration_node = Node(
        package='isaac_ros_nav2_integration',
        executable='vslam_odometry_fusion',
        name='vslam_odometry_fusion',
        parameters=[configured_params],
        remappings=[
            ('/input_odometry', '/vslam/odometry'),
            ('/output_odometry', '/fused_odometry'),
        ],
        output='screen',
        # Jetson-specific environment variables
        additional_env={
            'CUDA_DEVICE_ORDER': 'PCI_BUS_ID',
            'CUDA_VISIBLE_DEVICES': '0',
            'RCUTILS_LOGGING_BUFFERED_STREAM': '1'
        }
    )

    # Isaac ROS Footstep planner node (Jetson optimized)
    footstep_planner_node = Node(
        package='isaac_ros_footstep_planner',
        executable='footstep_planner_node',
        name='footstep_planner',
        parameters=[configured_params],
        remappings=[
            ('/input_path', '/plan'),
            ('/output_footsteps', '/footstep_path'),
        ],
        output='screen',
        # Jetson-specific environment variables
        additional_env={
            'CUDA_DEVICE_ORDER': 'PCI_BUS_ID',
            'CUDA_VISIBLE_DEVICES': '0',
            'RCUTILS_LOGGING_BUFFERED_STREAM': '1'
        }
    )

    # Isaac ROS Recovery behaviors node (Jetson optimized)
    recovery_behaviors_node = Node(
        package='isaac_ros_nav2_recovery',
        executable='recovery_behaviors_node',
        name='recovery_behaviors',
        parameters=[configured_params],
        output='screen',
        # Jetson-specific environment variables
        additional_env={
            'RCUTILS_LOGGING_BUFFERED_STREAM': '1'
        }
    )

    # Jetson-specific thermal monitoring
    thermal_monitor_node = Node(
        package='isaac_ros_nav2_jetson_utils',
        executable='thermal_monitor',
        name='jetson_thermal_monitor',
        parameters=[{
            'temperature_threshold': 85.0,  # Celsius
            'throttle_on_high_temp': True,
            'shutdown_on_critical_temp': False,  # Don't shut down, just reduce performance
            'critical_temperature': 95.0,  # Celsius
        }],
        condition=IfCondition(LaunchConfiguration('enable_thermal_monitoring', default='true'))
    )

    # Jetson power mode configuration
    power_mode_node = Node(
        package='isaac_ros_nav2_jetson_utils',
        executable='jetson_power_manager',
        name='jetson_power_manager',
        parameters=[{
            'target_power_mode': jetson_power_mode,
            'performance_scaling': 0.8,  # Scale performance for thermal management
        }],
        condition=IfCondition(LaunchConfiguration('enable_power_management', default='true'))
    )

    # Add nodes to launch description
    nodes = [
        # Launch Arguments
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_use_composition_cmd,
        declare_use_respawn_cmd,
        declare_jetson_power_mode_cmd,

        # Core Nav2 nodes
        nav2_container,
        amcl_node,
        planner_server_node,
        controller_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        lifecycle_manager_node,

        # Isaac ROS specific nodes
        isaac_ros_vslam_integration_node,
        footstep_planner_node,
        recovery_behaviors_node,

        # Jetson-specific nodes
        thermal_monitor_node,
        power_mode_node,
    ]

    return LaunchDescription(nodes)


if __name__ == '__main__':
    generate_launch_description()