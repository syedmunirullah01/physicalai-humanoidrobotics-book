#!/usr/bin/env python3
"""
Exercise 3: Map Relocalization
Chapter 2: Isaac ROS VSLAM - Map Saving, Loading, and Relocalization

Learning Objective: Implement map saving/loading and relocalization capabilities
FR-011: System MUST support saving/loading maps and relocalizing within previously mapped environments
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
import numpy as np
import cv2
from cv_bridge import CvBridge
import pickle
import json
import os
import time
from datetime import datetime
import threading


class VSLAMRelocalizationNode(Node):
    """
    VSLAM Relocalization node implementing map save/load and relocalization
    """

    def __init__(self):
        super().__init__('vslam_relocalization_node')

        # Initialize components
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # State variables
        self.current_pose = np.eye(4)  # Current pose estimate
        self.current_features = None
        self.current_descriptors = None
        self.map_database = {}  # Store map data: {keyframe_id: {'pose': ..., 'features': ..., 'descriptors': ...}}
        self.map_loaded = False
        self.is_localizing = False
        self.relocalization_active = False

        # Relocalization parameters
        self.relocalization_threshold = 0.7  # Similarity threshold for relocalization
        self.min_matches_for_relocalization = 20  # Minimum matches for valid relocalization
        self.relocalization_search_radius = 5.0  # Search radius for nearby keyframes (meters)

        # Map management
        self.map_directory = '/home/user/maps/vslam_maps/'
        self.current_map_name = None

        # Feature matching for relocalization
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_rect',
            self.image_callback,
            10
        )

        # Publishers
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/vslam/relocalization/pose',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/vslam/relocalization/status',
            10
        )

        self.relocalization_result_pub = self.create_publisher(
            Bool,
            '/vslam/relocalization/result',
            10
        )

        # Services for map management
        self.save_map_service = self.create_service(
            Trigger,
            '/vslam/save_map',
            self.save_map_callback
        )

        self.load_map_service = self.create_service(
            Trigger,
            '/vslam/load_map',
            self.load_map_callback
        )

        self.relocalize_service = self.create_service(
            Trigger,
            '/vslam/relocalize',
            self.relocalize_callback
        )

        self.reset_service = self.create_service(
            Trigger,
            '/vslam/reset',
            self.reset_callback
        )

        # Timers
        self.localization_timer = self.create_timer(0.1, self.localization_callback)  # 10 Hz localization

        self.get_logger().info('VSLAM Relocalization node initialized')

    def image_callback(self, msg):
        """
        Process incoming images for relocalization
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Extract features from current image
            features, descriptors = self.extract_features(cv_image)

            if features is not None and descriptors is not None:
                self.current_features = features
                self.current_descriptors = descriptors

                # If relocalization is active, attempt to relocalize
                if self.relocalization_active and self.map_loaded:
                    success, transform = self.attempt_relocalization(features, descriptors)

                    if success:
                        self.get_logger().info('✅ Relocalization successful!')

                        # Update current pose with relocalization result
                        self.current_pose = transform

                        # Publish relocalization success
                        result_msg = Bool()
                        result_msg.data = True
                        self.relocalization_result_pub.publish(result_msg)

                        # Update status
                        status_msg = String()
                        status_msg.data = f'RELOCALIZED: Pose updated with transform'
                        self.status_pub.publish(status_msg)

                        # Stop relocalization mode
                        self.relocalization_active = False
                    else:
                        # Continue searching
                        status_msg = String()
                        status_msg.data = f'SEARCHING: Attempting relocalization...'
                        self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image for relocalization: {e}')

    def extract_features(self, image):
        """
        Extract ORB features for relocalization
        """
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        # Use ORB for feature extraction (good for relocalization)
        orb = cv2.ORB_create(
            nfeatures=1000,
            scaleFactor=1.2,
            nlevels=8,
            edgeThreshold=31,
            firstLevel=0,
            WTA_K=2,
            scoreType=cv2.ORB_HARRIS_SCORE,
            patchSize=31,
            fastThreshold=20
        )

        keypoints, descriptors = orb.detectAndCompute(gray, None)

        if keypoints is not None and descriptors is not None:
            # Convert keypoints to numpy array for easier processing
            features = np.array([kp.pt for kp in keypoints])
            return features, descriptors
        else:
            return None, None

    def attempt_relocalization(self, current_features, current_descriptors):
        """
        Attempt to relocalize in the loaded map
        """
        if not self.map_database or current_descriptors is None:
            return False, None

        best_match_score = 0
        best_transform = None
        best_matches = []

        # Compare with all keyframes in the map
        for kf_id, kf_data in self.map_database.items():
            if kf_data['descriptors'] is None:
                continue

            # Match features with this keyframe
            matches = self.matcher.knnMatch(
                current_descriptors, kf_data['descriptors'], k=2
            )

            # Apply Lowe's ratio test
            good_matches = []
            for match_pair in matches:
                if len(match_pair) == 2:
                    m, n = match_pair
                    if m.distance < 0.75 * n.distance:
                        good_matches.append(m)

            # Check if this is a good candidate
            if len(good_matches) >= self.min_matches_for_relocalization:
                # Calculate similarity score
                score = len(good_matches) / max(len(current_descriptors), len(kf_data['descriptors']))

                if score > best_match_score and score > self.relocalization_threshold:
                    # Perform geometric verification
                    if len(good_matches) >= 10:  # Need minimum for RANSAC
                        # Get matched points
                        current_pts = np.float32([
                            current_features[m.queryIdx] for m in good_matches
                        ]).reshape(-1, 1, 2)
                        map_pts = np.float32([
                            kf_data['features'][m.trainIdx] for m in good_matches
                        ]).reshape(-1, 1, 2)

                        # Find essential matrix using RANSAC
                        E, mask = cv2.findEssentialMat(
                            current_pts, map_pts,
                            self.camera_matrix,  # Would need to store this in map
                            method=cv2.RANSAC,
                            prob=0.999,
                            threshold=1.0
                        )

                        if E is not None:
                            # Recover pose
                            points, R, t, mask_pose = cv2.recoverPose(
                                E, current_pts, map_pts, self.camera_matrix, mask
                            )

                            # Calculate transform from keyframe pose to current pose
                            kf_pose = kf_data['pose']
                            current_from_kf = np.eye(4)
                            current_from_kf[:3, :3] = R
                            current_from_kf[:3, 3] = t.flatten()

                            # Calculate current pose in map frame
                            current_pose = kf_pose @ current_from_kf

                            best_match_score = score
                            best_transform = current_pose
                            best_matches = good_matches

        if best_transform is not None:
            self.get_logger().info(
                f'Relocalization candidate found: score={best_match_score:.3f}, '
                f'matches={len(best_matches)}, keyframe={list(self.map_database.keys())[0]}'
            )
            return True, best_transform

        return False, None

    def save_map_callback(self, request, response):
        """
        Service callback to save current map to file
        """
        try:
            # Create map data structure
            map_data = {
                'metadata': {
                    'version': '1.0',
                    'saved_at': datetime.now().isoformat(),
                    'keyframe_count': len(self.map_database),
                    'map_name': self.current_map_name or f'map_{datetime.now().strftime("%Y%m%d_%H%M%S")}',
                    'total_path_length': self.calculate_path_length(),
                    'bounding_box': self.calculate_bounding_box()
                },
                'keyframes': {},
                'camera_calibration': self.get_camera_calibration()  # Would store actual calibration
            }

            # Add keyframe data
            for kf_id, kf_data in self.map_database.items():
                map_data['keyframes'][kf_id] = {
                    'pose': kf_data['pose'].tolist(),  # Convert numpy array to list
                    'feature_count': len(kf_data['features']) if kf_data['features'] is not None else 0,
                    'descriptor_shape': kf_data['descriptors'].shape if kf_data['descriptors'] is not None else [0, 0],
                    # Note: We don't store raw features/descriptors in map for size efficiency
                    # In real implementation, we'd store feature locations and use descriptors for matching
                }

            # Create directory if it doesn't exist
            os.makedirs(self.map_directory, exist_ok=True)

            # Generate filename
            map_name = self.current_map_name or f'vslam_map_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
            filepath = os.path.join(self.map_directory, f'{map_name}.vslam_map')

            # Save map data
            with open(filepath, 'wb') as f:
                pickle.dump(map_data, f)

            self.get_logger().info(f'Map saved successfully to {filepath}')

            response.success = True
            response.message = f'Map saved to {filepath} with {len(self.map_database)} keyframes'

        except Exception as e:
            self.get_logger().error(f'Error saving map: {e}')
            response.success = False
            response.message = f'Error saving map: {str(e)}'

        return response

    def load_map_callback(self, request, response):
        """
        Service callback to load map from file
        """
        try:
            # Find the most recent map file
            map_files = self.find_available_maps()
            if not map_files:
                response.success = False
                response.message = 'No map files found to load'
                return response

            # Load the most recent map
            latest_map_path = max(map_files, key=os.path.getctime)

            with open(latest_map_path, 'rb') as f:
                map_data = pickle.load(f)

            # Load keyframe data
            self.map_database = {}
            for kf_id, kf_info in map_data['keyframes'].items():
                self.map_database[kf_id] = {
                    'pose': np.array(kf_info['pose']),  # Convert back to numpy
                    'features': None,  # Would load from separate files in real implementation
                    'descriptors': None  # Would load from separate files in real implementation
                }

            # Set current map name
            self.current_map_name = os.path.basename(latest_map_path).replace('.vslam_map', '')
            self.map_loaded = True

            self.get_logger().info(
                f'Map loaded successfully: {latest_map_path} '
                f'with {len(self.map_database)} keyframes'
            )

            response.success = True
            response.message = f'Map loaded with {len(self.map_database)} keyframes'

        except Exception as e:
            self.get_logger().error(f'Error loading map: {e}')
            response.success = False
            response.message = f'Error loading map: {str(e)}'

        return response

    def relocalize_callback(self, request, response):
        """
        Service callback to initiate relocalization
        """
        if not self.map_loaded:
            response.success = False
            response.message = 'No map loaded - cannot relocalize'
            return response

        if self.current_descriptors is None:
            response.success = False
            response.message = 'No current image data available for relocalization'
            return response

        # Start relocalization process
        self.relocalization_active = True
        self.get_logger().info('Relocalization initiated - searching for location in loaded map')

        response.success = True
        response.message = 'Relocalization search started - monitoring for matches'

        return response

    def reset_callback(self, request, response):
        """
        Service callback to reset VSLAM system
        """
        # Reset all state
        self.current_pose = np.eye(4)
        self.current_features = None
        self.current_descriptors = None
        self.map_database = {}
        self.map_loaded = False
        self.is_localizing = False
        self.relocalization_active = False
        self.current_map_name = None

        self.get_logger().info('VSLAM system reset to initial state')

        response.success = True
        response.message = 'VSLAM system reset successfully'

        return response

    def find_available_maps(self):
        """
        Find all available map files in the map directory
        """
        if not os.path.exists(self.map_directory):
            return []

        map_files = []
        for filename in os.listdir(self.map_directory):
            if filename.endswith('.vslam_map'):
                filepath = os.path.join(self.map_directory, filename)
                map_files.append(filepath)

        return map_files

    def calculate_path_length(self):
        """
        Calculate total path length from keyframe poses
        """
        if len(self.map_database) < 2:
            return 0.0

        total_length = 0.0
        poses = [data['pose'] for data in self.map_database.values()]

        for i in range(1, len(poses)):
            prev_pos = poses[i-1][:3, 3]
            curr_pos = poses[i][:3, 3]
            dist = np.linalg.norm(curr_pos - prev_pos)
            total_length += dist

        return total_length

    def calculate_bounding_box(self):
        """
        Calculate bounding box of the map
        """
        if not self.map_database:
            return [[0, 0, 0], [0, 0, 0]]

        poses = [data['pose'] for data in self.map_database.values()]
        positions = np.array([pose[:3, 3] for pose in poses])

        min_pos = np.min(positions, axis=0)
        max_pos = np.max(positions, axis=0)

        return [min_pos.tolist(), max_pos.tolist()]

    def get_camera_calibration(self):
        """
        Get camera calibration parameters
        In real implementation, this would come from camera info topic
        """
        # Placeholder values - would be obtained from camera_info topic in real implementation
        return {
            'camera_matrix': [[426.67, 0.0, 424.0], [0.0, 426.67, 240.0], [0.0, 0.0, 1.0]],
            'distortion_coeffs': [0.0, 0.0, 0.0, 0.0, 0.0],
            'resolution': [848, 480]
        }

    def localization_callback(self):
        """
        Periodic localization callback
        """
        if self.map_loaded and not self.relocalization_active:
            # Publish current pose if available
            if self.current_pose is not None:
                self.publish_current_pose()

    def publish_current_pose(self):
        """
        Publish current pose estimate
        """
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        # Set position
        pos = self.current_pose[:3, 3]
        pose_msg.pose.position.x = float(pos[0])
        pose_msg.pose.position.y = float(pos[1])
        pose_msg.pose.position.z = float(pos[2])

        # Set orientation (convert rotation matrix to quaternion)
        R = self.current_pose[:3, :3]
        qw = np.sqrt(1 + R[0,0] + R[1,1] + R[2,2]) / 2
        qx = (R[2,1] - R[1,2]) / (4 * qw)
        qy = (R[0,2] - R[2,0]) / (4 * qw)
        qz = (R[1,0] - R[0,1]) / (4 * qw)

        pose_msg.pose.orientation.w = float(qw)
        pose_msg.pose.orientation.x = float(qx)
        pose_msg.pose.orientation.y = float(qy)
        pose_msg.pose.orientation.z = float(qz)

        self.pose_pub.publish(pose_msg)

        # Broadcast TF transform
        self.broadcast_transform(pose_msg)

    def broadcast_transform(self, pose_msg):
        """
        Broadcast TF transform for visualization
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'vslam_camera'

        # Set translation
        t.transform.translation.x = pose_msg.pose.position.x
        t.transform.translation.y = pose_msg.pose.position.y
        t.transform.translation.z = pose_msg.pose.position.z

        # Set rotation
        t.transform.rotation = pose_msg.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def get_map_info(self):
        """
        Get information about the current map
        """
        if not self.map_loaded:
            return "No map loaded"

        info = f"""
        Map Information:
        - Loaded: {self.map_loaded}
        - Map Name: {self.current_map_name}
        - Keyframes: {len(self.map_database)}
        - Path Length: {self.calculate_path_length():.2f}m
        - Bounding Box: {self.calculate_bounding_box()}
        - Directory: {self.map_directory}
        """

        return info


def main(args=None):
    """
    Main function to run the VSLAM relocalization node
    """
    rclpy.init(args=args)
    relocalization_node = VSLAMRelocalizationNode()

    try:
        rclpy.spin(relocalization_node)
    except KeyboardInterrupt:
        relocalization_node.get_logger().info('Shutting down VSLAM Relocalization node...')
    finally:
        relocalization_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()