#!/usr/bin/env python3

"""
Launch file for Isaac ROS Nav2 in simulation environment
Chapter 3: Isaac ROS Nav2 - Simulation Launch
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

    # Launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('isaac_ros_nav2'),
            'config', 'nav2_params.yaml'
        ]),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Whether to use composed bringup'
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes'
    )

    # Set environment variables
    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']

    # Replace with Isaac ROS VSLAM odometry
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('/cmd_vel', 'cmd_vel'),
                  ('/odom', '/vslam/odometry'),  # Use VSLAM odometry
                  ('/initialpose', 'initialpose'),
                  ('/goal_pose', 'goal_pose'),
                  ('/map', 'map'),
                  ('/scan', 'scan'),
                  ('/global_costmap/costmap_raw', 'global_costmap/costmap_raw'),
                  ('/global_costmap/costmap', 'global_costmap/costmap'),
                  ('/global_costmap/costmap_updates', 'global_costmap/costmap_updates'),
                  ('/local_costmap/costmap_raw', 'local_costmap/costmap_raw'),
                  ('/local_costmap/costmap', 'local_costmap/costmap'),
                  ('/local_costmap/costmap_updates', 'local_costmap/costmap_updates'),
                  ('/global_costmap/global_costmap/voxel_marked_cloud', 'global_costmap/voxel_marked_cloud')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    # Navigation server (composed)
    nav2_container = Node(
        condition=IfCondition(use_composition),
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[configured_params, {'autostart': autostart}],
        remappings=remappings,
        output='screen'
    )

    # AMCL node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[configured_params],
        remappings=remappings,
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0
    )

    # Planner Server
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

    # Controller Server
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

    # Behavior Server
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

    # BT Navigator
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

    # Waypoint Follower
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

    # Velocity Smoother
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        parameters=[configured_params],
        remappings=remappings,
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0
    )

    # Lifecycle Manager
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                   {'autostart': autostart},
                   {'node_names': lifecycle_nodes}]
    )

    # Isaac ROS VSLAM integration node
    isaac_ros_vslam_integration_node = Node(
        package='isaac_ros_nav2_integration',
        executable='vslam_odometry_fusion',
        name='vslam_odometry_fusion',
        parameters=[configured_params],
        remappings=[
            ('/input_odometry', '/vslam/odometry'),
            ('/output_odometry', '/fused_odometry'),
        ],
        output='screen'
    )

    # Isaac ROS Footstep planner node
    footstep_planner_node = Node(
        package='isaac_ros_footstep_planner',
        executable='footstep_planner_node',
        name='footstep_planner',
        parameters=[configured_params],
        remappings=[
            ('/input_path', '/plan'),
            ('/output_footsteps', '/footstep_path'),
        ],
        output='screen'
    )

    # Isaac ROS Recovery behaviors node
    recovery_behaviors_node = Node(
        package='isaac_ros_nav2_recovery',
        executable='recovery_behaviors_node',
        name='recovery_behaviors',
        parameters=[configured_params],
        output='screen'
    )

    # Add nodes to launch description
    nodes = [
        # Launch Arguments
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_use_composition_cmd,
        declare_use_respawn_cmd,

        # Core Nav2 nodes
        nav2_container,
        amcl_node,
        planner_server_node,
        controller_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        velocity_smoother_node,
        lifecycle_manager_node,

        # Isaac ROS specific nodes
        isaac_ros_vslam_integration_node,
        footstep_planner_node,
        recovery_behaviors_node,
    ]

    return LaunchDescription(nodes)


if __name__ == '__main__':
    generate_launch_description()