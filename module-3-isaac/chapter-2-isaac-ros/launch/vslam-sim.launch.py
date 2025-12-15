#!/usr/bin/env python3

"""
Launch file for Isaac ROS VSLAM in simulation environment
Chapter 2: Isaac ROS VSLAM - Simulation Launch
"""

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    enable_loop_closure_arg = DeclareLaunchArgument(
        'enable_loop_closure',
        default_value='true',
        description='Enable loop closure detection for drift correction'
    )

    enable_debug_mode_arg = DeclareLaunchArgument(
        'enable_debug_mode',
        default_value='false',
        description='Enable debug mode with additional visualizations'
    )

    enable_mapping_arg = DeclareLaunchArgument(
        'enable_mapping',
        default_value='true',
        description='Enable 3D map building'
    )

    # Isaac ROS Visual SLAM node
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='isaac_ros_visual_slam_node',
        name='visual_slam',
        parameters=[{
            # Enable rectified pose output
            'enable_rectified_pose': True,

            # Enable debug mode for visualization
            'enable_debug_mode': LaunchConfiguration('enable_debug_mode'),

            # Enable SLAM visualization
            'enable_slam_visualization': True,

            # Enable landmarks view for debugging
            'enable_landmarks_view': True,

            # Enable observations view (disable for performance)
            'enable_observations_view': False,

            # Frame IDs for TF tree
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'camera_frame': 'camera_link',

            # Number of cameras (2 for stereo VSLAM)
            'num_cameras': 2,

            # Minimum number of images for initialization
            'min_num_images': 30,

            # Maximum frame rate for processing
            'max_frame_rate': 60.0,

            # Feature detector parameters
            'feature_detector_type': 'ORB',
            'num_features': 1000,
            'scale_factor': 1.2,
            'num_levels': 8,
            'edge_threshold': 19,
            'wta_k': 2,
            'score_type': 0,  # HARRIS_SCORE
            'patch_size': 31,
            'fast_threshold': 20,

            # Motion estimation parameters
            'reproj_err_threshold': 3.0,
            'min_num_inliers': 15,
            'ransac_confidence': 0.99,
            'ransac_max_iter': 1000,

            # Loop closure parameters
            'enable_loop_closure': LaunchConfiguration('enable_loop_closure'),
            'loop_closure_frequency': 1.0,  # Hz
            'loop_closure_min_score': 0.7,
            'enable_pose_graph_optimizer': True,
            'pose_graph_optimizer_frequency': 1.0,  # Hz

            # Bundle adjustment parameters
            'enable_global_ba': True,
            'global_ba_frequency': 5,  # Run every 5 keyframes
            'global_ba_max_num_iterations': 100,
            'enable_local_ba': True,
            'local_ba_max_num_iterations': 20,

            # Keyframe selection parameters
            'min_translation_keyframe_delta': 0.1,
            'min_rotation_keyframe_delta': 0.1,
            'min_num_features_for_tracking': 30,

            # GPU acceleration parameters
            'gpu_id': 0,
            'enable_gpu_acceleration': True,
        }],
        remappings=[
            # Input remappings for stereo camera
            ('/visual_slam/image_0', '/camera/left/image_rect'),
            ('/visual_slam/camera_info_0', '/camera/left/camera_info'),
            ('/visual_slam/image_1', '/camera/right/image_rect'),
            ('/visual_slam/camera_info_1', '/camera/right/camera_info'),

            # Output remappings
            ('/visual_slam/tracking/odometry', '/vslam/odometry'),
            ('/visual_slam/tracking/vo_pose', '/vslam/vo_pose'),
            ('/visual_slam/tracking/vo_pose_covariance', '/vslam/vo_pose_covariance'),

            # Visualization remappings
            ('/visual_slam/vis/landmarks_cloud', '/vslam/landmarks'),
            ('/visual_slam/vis/loop_closure_cloud', '/vslam/loop_closure'),
            ('/visual_slam/vis/pose_graph_nodes', '/vslam/pose_graph_nodes'),
            ('/visual_slam/vis/pose_graph_edges', '/vslam/pose_graph_edges'),
            ('/visual_slam/vis/keyframes', '/vslam/keyframes'),
        ]
    )

    # Optional: Add stereo image processing node for rectification
    stereo_rectify_node = Node(
        package='stereo_image_proc',
        executable='stereo_image_proc',
        name='stereo_rectify',
        parameters=[
            {'approximate_sync': True},
            {'queue_size': 10}
        ],
        remappings=[
            ('left/image_raw', '/camera/left/image_raw'),
            ('left/camera_info', '/camera/left/camera_info'),
            ('right/image_raw', '/camera/right/image_raw'),
            ('right/camera_info', '/camera/right/camera_info'),
            ('left/image_rect', '/camera/left/image_rect'),
            ('right/image_rect', '/camera/right/image_rect'),
        ],
        condition=IfCondition(LaunchConfiguration('enable_rectification', default='true'))
    )

    # Optional: Add RViz2 for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('isaac_ros_visual_slam'),
            'rviz', 'visual_slam.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('enable_rviz', default='false'))
    )

    return LaunchDescription([
        enable_loop_closure_arg,
        enable_debug_mode_arg,
        enable_mapping_arg,

        visual_slam_node,
        stereo_rectify_node,
        rviz_node,
    ])


if __name__ == '__main__':
    generate_launch_description()