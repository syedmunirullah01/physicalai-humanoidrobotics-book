#!/usr/bin/env python3
"""
Exercise 1: Visual Odometry with Isaac ROS
Chapter 2: Isaac ROS VSLAM - Visual Odometry

Learning Objective: Implement monocular visual odometry using Isaac ROS cuVSLAM
FR-007: System MUST implement visual odometry for pose estimation from camera inputs
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import cv2
from tf2_ros import TransformBroadcaster
import tf_transformations


class VisualOdometryNode(Node):
    """
    Visual Odometry node implementing FR-007 requirements
    """

    def __init__(self):
        super().__init__('visual_odometry_node')

        # Initialize CV bridge for image processing
        self.bridge = CvBridge()

        # Initialize state variables
        self.current_pose = np.eye(4)  # 4x4 transformation matrix
        self.previous_image = None
        self.current_image = None
        self.camera_matrix = None
        self.distortion_coeffs = None
        self.frame_count = 0
        self.initialized = False

        # Feature tracking parameters
        self.feature_params = {
            'max_corners': 1000,
            'quality_level': 0.01,
            'min_distance': 10,
            'block_size': 3
        }

        # Lucas-Kanade optical flow parameters
        self.lk_params = {
            'winSize': (15, 15),
            'maxLevel': 2,
            'criteria': (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
        }

        # Pose estimation parameters
        self.min_features_for_tracking = 50
        self.reprojection_error_threshold = 3.0

        # Subscribers for camera data
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_rect',  # Rectified image topic
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Publishers for odometry and pose
        self.odom_pub = self.create_publisher(
            Odometry,
            '/visual_odometry/odometry',
            10
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/visual_odometry/pose',
            10
        )

        # TF broadcaster for pose visualization
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for processing
        self.timer = self.create_timer(0.033, self.process_vo)  # ~30 Hz

        self.get_logger().info('Visual Odometry node initialized for FR-007')

    def camera_info_callback(self, msg):
        """
        Callback to receive camera calibration information
        """
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.distortion_coeffs = np.array(msg.d)
            self.get_logger().info(f'Camera calibration received: {self.camera_matrix[0,0]:.1f} fx')

    def image_callback(self, msg):
        """
        Callback to process incoming images for visual odometry
        """
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Convert to grayscale if needed
            if len(cv_image.shape) == 3:
                gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            else:
                gray = cv_image

            self.current_image = gray

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def extract_features(self, image):
        """
        Extract features from image using Shi-Tomasi corner detection
        """
        if image is None:
            return np.array([])

        # Use Shi-Tomasi corner detection
        features = cv2.goodFeaturesToTrack(
            image,
            maxCorners=self.feature_params['max_corners'],
            qualityLevel=self.feature_params['quality_level'],
            minDistance=self.feature_params['min_distance'],
            blockSize=self.feature_params['block_size']
        )

        return features

    def estimate_motion(self, prev_img, curr_img, prev_features):
        """
        Estimate motion between two frames using optical flow
        """
        if prev_features is None or len(prev_features) < 10:
            return None, 0

        # Calculate optical flow using Lucas-Kanade
        curr_features, status, error = cv2.calcOpticalFlowPyrLK(
            prev_img, curr_img,
            prev_features, None,
            **self.lk_params
        )

        # Filter out bad matches
        good_prev = prev_features[status.ravel() == 1]
        good_curr = curr_features[status.ravel() == 1]

        if len(good_prev) < 10:
            return None, 0

        # Estimate essential matrix using RANSAC
        essential_matrix, mask = cv2.findEssentialMat(
            good_curr, good_prev,
            cameraMatrix=self.camera_matrix,
            method=cv2.RANSAC,
            prob=0.999,
            threshold=self.reprojection_error_threshold
        )

        if essential_matrix is None or len(essential_matrix) == 0:
            return None, 0

        # Recover pose from essential matrix
        _, rotation, translation, mask_pose = cv2.recoverPose(
            essential_matrix, good_curr, good_prev,
            cameraMatrix=self.camera_matrix, mask=mask
        )

        # Convert rotation matrix to quaternion
        rotation_matrix = np.eye(4)
        rotation_matrix[:3, :3] = rotation
        quaternion = self.rotation_matrix_to_quaternion(rotation_matrix[:3, :3])

        # Create transformation matrix
        transform = np.eye(4)
        transform[:3, :3] = rotation
        transform[:3, 3] = translation.flatten()

        return transform, len(good_curr)

    def rotation_matrix_to_quaternion(self, rotation_matrix):
        """
        Convert rotation matrix to quaternion
        """
        # Using the algorithm from Hartley & Zisserman
        R = rotation_matrix
        trace = np.trace(R)

        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2  # s = 4 * qw
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # s = 4 * qx
                qw = (R[2, 1] - R[1, 2]) / s
                qx = 0.25 * s
                qy = (R[0, 1] + R[1, 0]) / s
                qz = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # s = 4 * qy
                qw = (R[0, 2] - R[2, 0]) / s
                qx = (R[0, 1] + R[1, 0]) / s
                qy = 0.25 * s
                qz = (R[1, 2] + R[2, 1]) / s
            else:
                s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # s = 4 * qz
                qw = (R[1, 0] - R[0, 1]) / s
                qx = (R[0, 2] + R[2, 0]) / s
                qy = (R[1, 2] + R[2, 1]) / s
                qz = 0.25 * s

        return np.array([qw, qx, qy, qz])

    def process_vo(self):
        """
        Main visual odometry processing loop
        """
        if (self.current_image is None or self.camera_matrix is None or
            self.previous_image is None):
            return

        # Extract features from current image
        current_features = self.extract_features(self.current_image)

        if current_features is not None and len(current_features) > 0:
            # Estimate motion between previous and current frame
            motion_transform, num_inliers = self.estimate_motion(
                self.previous_image,
                self.current_image,
                current_features
            )

            if motion_transform is not None and num_inliers >= self.min_features_for_tracking:
                # Update current pose by applying motion transform
                self.current_pose = self.current_pose @ motion_transform

                # Publish odometry and pose
                self.publish_odometry()
                self.publish_pose()

                # Update frame counter
                self.frame_count += 1

                if self.frame_count % 30 == 0:  # Log every 30 frames
                    pos = self.current_pose[:3, 3]
                    self.get_logger().info(
                        f'VO Update #{self.frame_count}: '
                        f'Pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}), '
                        f'Features={num_inliers}, '
                        f'Translation={np.linalg.norm(motion_transform[:3, 3]):.3f}m'
                    )

        # Update previous image for next iteration
        self.previous_image = self.current_image.copy()

    def publish_odometry(self):
        """
        Publish odometry message with pose and twist information
        """
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'

        # Set position
        pos = self.current_pose[:3, 3]
        odom_msg.pose.pose.position.x = float(pos[0])
        odom_msg.pose.pose.position.y = float(pos[1])
        odom_msg.pose.pose.position.z = float(pos[2])

        # Set orientation (from rotation matrix to quaternion)
        R = self.current_pose[:3, :3]
        quat = self.rotation_matrix_to_quaternion(R)
        odom_msg.pose.pose.orientation.w = float(quat[0])
        odom_msg.pose.pose.orientation.x = float(quat[1])
        odom_msg.pose.pose.orientation.y = float(quat[2])
        odom_msg.pose.pose.orientation.z = float(quat[3])

        # Set zero velocity for now (would come from differentiation in real implementation)
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom_msg)

    def publish_pose(self):
        """
        Publish pose stamped message
        """
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        # Set position
        pos = self.current_pose[:3, 3]
        pose_msg.pose.position.x = float(pos[0])
        pose_msg.pose.position.y = float(pos[1])
        pose_msg.pose.position.z = float(pos[2])

        # Set orientation
        R = self.current_pose[:3, :3]
        quat = self.rotation_matrix_to_quaternion(R)
        pose_msg.pose.orientation.w = float(quat[0])
        pose_msg.pose.orientation.x = float(quat[1])
        pose_msg.pose.orientation.y = float(quat[2])
        pose_msg.pose.orientation.z = float(quat[3])

        self.pose_pub.publish(pose_msg)

    def broadcast_transform(self):
        """
        Broadcast TF transform for visualization
        """
        from geometry_msgs.msg import TransformStamped

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        # Set translation
        pos = self.current_pose[:3, 3]
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])

        # Set rotation
        R = self.current_pose[:3, :3]
        quat = self.rotation_matrix_to_quaternion(R)
        t.transform.rotation.w = float(quat[0])
        t.transform.rotation.x = float(quat[1])
        t.transform.rotation.y = float(quat[2])
        t.transform.rotation.z = float(quat[3])

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    """
    Main function to run the visual odometry node
    """
    rclpy.init(args=args)

    vo_node = VisualOdometryNode()

    try:
        rclpy.spin(vo_node)
    except KeyboardInterrupt:
        vo_node.get_logger().info('Shutting down Visual Odometry node...')
    finally:
        vo_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()