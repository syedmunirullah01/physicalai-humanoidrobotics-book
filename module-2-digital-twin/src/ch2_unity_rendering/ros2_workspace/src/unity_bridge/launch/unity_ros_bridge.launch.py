#!/usr/bin/env python3
"""
Launch file for Unity ROS 2 bridge

This launch file starts the ROS-TCP-Endpoint server that Unity connects to.

Usage:
    ros2 launch unity_bridge unity_ros_bridge.launch.py

Configuration:
    - ROS_IP: IP address to bind to (0.0.0.0 = all interfaces)
    - ROS_TCP_PORT: TCP port for Unity connection (default: 10000)

After launching, Unity can connect via ROS-TCP-Connector.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Unity ROS bridge"""

    # Declare launch arguments
    ros_ip_arg = DeclareLaunchArgument(
        'ROS_IP',
        default_value='0.0.0.0',
        description='IP address to bind ROS-TCP-Endpoint (0.0.0.0 = all interfaces)'
    )

    ros_port_arg = DeclareLaunchArgument(
        'ROS_TCP_PORT',
        default_value='10000',
        description='TCP port for Unity connection'
    )

    # ROS-TCP-Endpoint server node
    tcp_endpoint_node = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='unity_tcp_endpoint',
        output='screen',
        parameters=[{
            'ROS_IP': LaunchConfiguration('ROS_IP'),
            'ROS_TCP_PORT': LaunchConfiguration('ROS_TCP_PORT'),
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Log startup information
    startup_log = LogInfo(
        msg=[
            '\n',
            '========================================\n',
            'Unity ROS Bridge Started\n',
            '========================================\n',
            'ROS-TCP-Endpoint listening on: ',
            LaunchConfiguration('ROS_IP'),
            ':',
            LaunchConfiguration('ROS_TCP_PORT'),
            '\n',
            '\n',
            'Unity Connection Instructions:\n',
            '1. Open Unity project\n',
            '2. Ensure ROSConnection.cs is attached to a GameObject\n',
            '3. Configure IP address in Inspector (localhost or this machine\'s IP)\n',
            '4. Play scene in Unity Editor\n',
            '5. Check Unity Console for "Connected to ROS 2" message\n',
            '\n',
            'Test Connection:\n',
            '  ros2 topic list  # Should show Unity-published topics\n',
            '  ros2 topic pub /joint_states sensor_msgs/msg/JointState "..."  # Test Unity subscription\n',
            '\n',
            'Troubleshooting:\n',
            '  - Firewall: Allow TCP port 10000\n',
            '  - Network: Ensure Unity machine can reach ROS machine IP\n',
            '  - ROS 2: Verify ros_tcp_endpoint package is installed\n',
            '========================================\n'
        ]
    )

    return LaunchDescription([
        ros_ip_arg,
        ros_port_arg,
        startup_log,
        tcp_endpoint_node,
    ])
