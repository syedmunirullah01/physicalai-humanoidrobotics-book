#!/usr/bin/env python3
"""
ROS 2 Launch File Template
Physical AI & Humanoid Robotics Textbook - Module 1

Purpose: Reusable template for launching multiple ROS 2 nodes, robot state publisher, RViz, and Gazebo

Usage:
1. Copy this template to your package's launch/ directory
2. Customize the nodes, parameters, and arguments
3. Launch with: ros2 launch <package_name> <launch_file>.py

Common Patterns Demonstrated:
- Launch multiple nodes simultaneously
- Load URDF from file and publish to robot_state_publisher
- Start RViz with custom configuration
- Spawn robot in Gazebo simulation
- Pass command-line arguments to launch file
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Generate the launch description with all nodes and configurations.

    Returns:
        LaunchDescription: Complete launch configuration
    """

    # ============================================================================
    # Launch Arguments (Command-line parameters)
    # ============================================================================

    # Declare argument for URDF file path (can be overridden from command line)
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='robot.urdf',  # Change to your URDF filename
        description='Name of URDF file to load (must be in urdf/ folder)'
    )

    # Declare argument for RViz configuration file
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='default.rviz',  # Change to your RViz config filename
        description='Name of RViz configuration file (must be in rviz/ folder)'
    )

    # Declare argument for Gazebo world
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.world',  # Change to your Gazebo world file
        description='Gazebo world file to load'
    )

    # Use joint state publisher GUI by default (set to false for headless)
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Use joint_state_publisher_gui for manual joint control'
    )

    # ============================================================================
    # File Paths (Replace 'your_package_name' with actual package name)
    # ============================================================================

    package_name = 'your_package_name'  # **CHANGE THIS**

    # Get package share directory
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    # URDF file path
    urdf_file = PathJoinSubstitution([
        pkg_share,
        'urdf',
        LaunchConfiguration('urdf_file')
    ])

    # RViz configuration file path
    rviz_config_file = PathJoinSubstitution([
        pkg_share,
        'rviz',
        LaunchConfiguration('rviz_config')
    ])

    # ============================================================================
    # Robot State Publisher Node
    # ============================================================================

    # Read URDF file (alternative: use xacro if you have complex URDFs)
    with open(os.path.join(pkg_share, 'urdf', 'robot.urdf'), 'r') as urdf_file_handle:
        robot_description = urdf_file_handle.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True  # Set to True if using Gazebo simulation
        }]
    )

    # ============================================================================
    # Joint State Publisher (GUI or non-GUI)
    # ============================================================================

    # Joint State Publisher GUI (for manual joint control with sliders)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=LaunchConfiguration('use_gui')  # Only launch if use_gui=true
    )

    # Joint State Publisher (non-GUI, for automated joint states)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=LaunchConfiguration('use_gui')  # Launch if use_gui=false (you'll need to add NOT logic)
    )

    # ============================================================================
    # RViz2 Node (3D Visualization)
    # ============================================================================

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],  # Load RViz config file
        parameters=[{'use_sim_time': True}]
    )

    # ============================================================================
    # Gazebo Simulation (Optional - Uncomment if using Gazebo)
    # ============================================================================

    # # Include Gazebo launch file
    # gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('gazebo_ros'),
    #             'launch',
    #             'gazebo.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={'world': LaunchConfiguration('world')}.items()
    # )

    # # Spawn robot in Gazebo
    # spawn_robot_node = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     name='spawn_robot',
    #     output='screen',
    #     arguments=[
    #         '-entity', 'robot_name',  # **CHANGE THIS**
    #         '-topic', '/robot_description',
    #         '-x', '0.0',
    #         '-y', '0.0',
    #         '-z', '0.5'
    #     ]
    # )

    # ============================================================================
    # Custom Node Example (Replace with your own nodes)
    # ============================================================================

    # custom_node = Node(
    #     package='your_package_name',
    #     executable='your_node_executable',  # Must match entry in setup.py or CMakeLists.txt
    #     name='custom_node',
    #     output='screen',
    #     parameters=[
    #         {'param1': 'value1'},
    #         {'param2': 42}
    #     ]
    # )

    # ============================================================================
    # Launch Description (Add all nodes here)
    # ============================================================================

    return LaunchDescription([
        # Arguments
        urdf_file_arg,
        rviz_config_arg,
        world_arg,
        use_gui_arg,

        # Nodes
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,

        # Uncomment if using Gazebo:
        # gazebo_launch,
        # spawn_robot_node,

        # Add your custom nodes here:
        # custom_node,
    ])


if __name__ == '__main__':
    generate_launch_description()
