#!/usr/bin/env python3

"""
Launch file for Isaac ROS VSLAM on Jetson Orin platforms
Chapter 2: Isaac ROS VSLAM - Jetson Deployment
"""

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    enable_loop_closure_arg = DeclareLaunchArgument(
        'enable_loop_closure',
        default_value='true',
        description='Enable loop closure detection'
    )

    enable_debug_mode_arg = DeclareLaunchArgument(
        'enable_debug_mode',
        default_value='false',
        description='Enable debug mode with additional visualizations'
    )

    power_mode_arg = DeclareLaunchArgument(
        'jetson_power_mode',
        default_value='MODE_15W',
        description='Jetson power mode: MODE_15W, MODE_MAXN, etc.'
    )

    # Isaac ROS Visual SLAM node (optimized for Jetson)
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='isaac_ros_visual_slam_node',
        name='visual_slam_jetson',
        parameters=[{
            # Input configuration (optimized for Jetson cameras)
            'enable_rectified_pose': True,
            'enable_debug_mode': LaunchConfiguration('enable_debug_mode'),
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': False,  # Disable for performance on Jetson

            # Frame IDs
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'camera_frame': 'camera_link',

            # Performance parameters (optimized for Jetson)
            'num_cameras': 2,  # Stereo configuration
            'min_num_images': 20,  # Reduced from 30 for Jetson
            'max_frame_rate': 25.0,  # Lower frame rate for Jetson (25 Hz)

            # Feature detection (optimized for Jetson)
            'feature_detector_type': 'ORB',
            'num_features': 800,  # Reduced from 1000 for Jetson
            'scale_factor': 1.2,
            'num_levels': 6,  # Reduced pyramid levels for performance
            'edge_threshold': 19,
            'wta_k': 2,
            'score_type': 0,  # HARRIS_SCORE
            'patch_size': 19,  # Smaller patch for speed
            'fast_threshold': 15,  # Lower threshold for more features

            # Motion estimation (optimized for Jetson)
            'reproj_err_threshold': 3.0,
            'min_num_inliers': 12,  # Reduced for Jetson
            'ransac_confidence': 0.99,
            'ransac_max_iter': 500,  # Reduced iterations for performance

            # Loop closure (optimized for Jetson)
            'enable_loop_closure': LaunchConfiguration('enable_loop_closure'),
            'loop_closure_frequency': 0.5,  # Less frequent on Jetson (every 2 seconds)
            'loop_closure_min_score': 0.6,  # Lower threshold for Jetson
            'enable_pose_graph_optimizer': True,
            'pose_graph_optimizer_frequency': 10,  # Every 10 seconds

            # Bundle adjustment (disabled for Jetson performance)
            'enable_global_ba': False,  # Disabled for Jetson
            'enable_local_ba': True,
            'local_ba_max_num_iterations': 10,  # Reduced iterations
            'local_ba_num_keyframes': 8,  # Reduced keyframes

            # Keyframe selection (optimized for Jetson)
            'min_translation_keyframe_delta': 0.15,  # Increased for Jetson
            'min_rotation_keyframe_delta': 0.15,    # Increased for Jetson
            'min_num_features_for_tracking': 25,     # Reduced for Jetson

            # GPU acceleration (specific to Jetson Orin)
            'gpu_id': 0,
            'enable_gpu_acceleration': True,
            'cuda_streams': 2,  # Reduced for Jetson memory constraints

            # Memory management (crucial for Jetson)
            'enable_memory_pool': True,
            'memory_pool_size': 1024 * 1024 * 64,  # 64 MB for Jetson
        }],
        remappings=[
            # Standard ROS 2 topic remappings for Jetson cameras
            ('/visual_slam/image_0', '/camera/left/image_rect'),
            ('/visual_slam/camera_info_0', '/camera/left/camera_info'),
            ('/visual_slam/image_1', '/camera/right/image_rect'),
            ('/visual_slam/camera_info_1', '/camera/right/camera_info'),

            # Output topics
            ('/visual_slam/tracking/odometry', '/vslam/odometry'),
            ('/visual_slam/tracking/vo_pose', '/vslam/vo_pose'),
            ('/visual_slam/tracking/vo_pose_covariance', '/vslam/vo_pose_covariance'),

            # Visualization topics
            ('/visual_slam/vis/landmarks_cloud', '/vslam/landmarks'),
            ('/visual_slam/vis/loop_closure_cloud', '/vslam/loop_closure'),
            ('/visual_slam/vis/pose_graph_nodes', '/vslam/pose_graph_nodes'),
            ('/visual_slam/vis/pose_graph_edges', '/vslam/pose_graph_edges'),
            ('/visual_slam/vis/keyframes', '/vslam/keyframes'),
        ],
        # Set environment variables specific to Jetson
        additional_env={
            'CUDA_DEVICE_ORDER': 'PCI_BUS_ID',
            'CUDA_VISIBLE_DEVICES': '0',
            'PYTHONUNBUFFERED': '1',
            'OMP_NUM_THREADS': '4',  # Limit OpenMP threads on Jetson
        }
    )

    # Optional: Jetson-specific thermal monitoring
    thermal_monitor_node = Node(
        package='isaac_ros_vslam_jetson_utils',
        executable='thermal_monitor',
        name='jetson_thermal_monitor',
        parameters=[{
            'temperature_threshold': 85.0,  # Celsius
            'throttle_on_high_temp': True,
            'shutdown_on_critical_temp': False,  # Don't shut down, just reduce performance
            'critical_temperature': 95.0,  # Celsius
        }],
        condition=launch.conditions.IfCondition('enable_thermal_monitoring', default='false')
    )

    # Jetson power mode configuration (if needed)
    power_mode_node = Node(
        package='isaac_ros_vslam_jetson_utils',
        executable='jetson_power_manager',
        name='jetson_power_manager',
        parameters=[{
            'target_power_mode': LaunchConfiguration('jetson_power_mode'),
            'performance_scaling': 0.8,  # Scale performance for thermal management
        }],
        condition=launch.conditions.IfCondition('enable_power_management', default='true')
    )

    return LaunchDescription([
        enable_loop_closure_arg,
        enable_debug_mode_arg,
        power_mode_arg,

        # Jetson-specific nodes
        power_mode_node,
        thermal_monitor_node,

        # Main VSLAM node
        visual_slam_node,
    ])


if __name__ == '__main__':
    generate_launch_description()