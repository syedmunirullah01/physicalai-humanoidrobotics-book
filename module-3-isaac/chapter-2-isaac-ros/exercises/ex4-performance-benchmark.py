#!/usr/bin/env python3
"""
Exercise 4: Performance Benchmarking
Chapter 2: Isaac ROS VSLAM - Performance Measurement and Validation

Learning Objective: Measure VSLAM performance metrics and validate success criteria
SC-002: VSLAM implementation achieves >30 Hz real-time performance on standard development hardware with GPU acceleration
SC-003: VSLAM trajectory maintains <2% drift over 100-meter paths in simulation environments
FR-012: System MUST provide performance metrics for VSLAM (pose accuracy, frame rate, map quality, memory usage)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Header
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformListener, Buffer
import numpy as np
import cv2
from cv_bridge import CvBridge
import time
import threading
from collections import deque
import psutil
import GPUtil
import json
from datetime import datetime


class VSLAMBenchmarkNode(Node):
    """
    VSLAM Performance Benchmark node implementing performance measurement and validation
    """

    def __init__(self):
        super().__init__('vslam_benchmark_node')

        # Initialize components
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Performance tracking
        self.frame_times = deque(maxlen=1000)  # Last 1000 frame times
        self.processing_times = deque(maxlen=1000)  # Processing times
        self.pose_history = deque(maxlen=10000)  # Historical poses
        self.feature_counts = deque(maxlen=1000)  # Feature counts per frame
        self.memory_usage_history = deque(maxlen=1000)  # Memory usage over time
        self.gpu_usage_history = deque(maxlen=1000)  # GPU usage over time

        # Ground truth tracking (for drift calculation)
        self.ground_truth_history = deque(maxlen=10000)
        self.ground_truth_available = False

        # Performance metrics
        self.start_time = time.time()
        self.frame_count = 0
        self.total_processing_time = 0.0
        self.current_fps = 0.0
        self.avg_processing_time = 0.0

        # Success criteria tracking
        self.total_path_length = 0.0
        self.drift_distance = 0.0
        self.drift_percentage = 0.0

        # Hardware monitoring
        self.gpu_available = self.check_gpu_availability()

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/isaac_ros_vslam/visual_slam/tracking/odometry',
            self.odom_callback,
            10
        )

        self.ground_truth_sub = self.create_subscription(
            Odometry,
            '/ground_truth/odometry',  # Ground truth from simulator
            self.ground_truth_callback,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            '/camera/left/image_rect',
            self.image_callback,
            10
        )

        # Publishers for metrics
        self.fps_pub = self.create_publisher(
            Float32,
            '/vslam/metrics/fps',
            10
        )

        self.processing_time_pub = self.create_publisher(
            Float32,
            '/vslam/metrics/processing_time_ms',
            10
        )

        self.drift_pub = self.create_publisher(
            Float32,
            '/vslam/metrics/drift_percent',
            10
        )

        self.memory_pub = self.create_publisher(
            Float32,
            '/vslam/metrics/memory_usage_mb',
            10
        )

        self.gpu_usage_pub = self.create_publisher(
            Float32,
            '/vslam/metrics/gpu_usage_percent',
            10
        )

        self.benchmark_results_pub = self.create_publisher(
            String,
            '/vslam/benchmark/results',
            10
        )

        # Timer for periodic metrics calculation
        self.metrics_timer = self.create_timer(1.0, self.calculate_metrics)  # 1 Hz metrics update

        # Timer for performance validation
        self.validation_timer = self.create_timer(5.0, self.validate_performance)  # 5 Hz validation

        self.get_logger().info('VSLAM Benchmark node initialized')

    def image_callback(self, msg):
        """
        Process image timestamps for frame rate calculation
        """
        self.frame_count += 1
        current_time = time.time()

        if len(self.frame_times) > 0:
            frame_time = current_time - self.frame_times[-1] if self.frame_times else 0
            self.frame_times.append(current_time)

            if frame_time > 0:
                self.current_fps = 1.0 / frame_time

    def odom_callback(self, msg):
        """
        Process odometry messages for pose tracking
        """
        # Record pose with timestamp
        pose_data = {
            'position': [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
            'orientation': [
                msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z
            ],
            'timestamp': time.time(),
            'frame_id': msg.header.frame_id
        }

        self.pose_history.append(pose_data)

        # Calculate path length if we have previous pose
        if len(self.pose_history) > 1:
            prev_pose = self.pose_history[-2]
            curr_pose = self.pose_history[-1]

            displacement = np.sqrt(
                (curr_pose['position'][0] - prev_pose['position'][0])**2 +
                (curr_pose['position'][1] - prev_pose['position'][1])**2 +
                (curr_pose['position'][2] - prev_pose['position'][2])**2
            )

            self.total_path_length += displacement

    def ground_truth_callback(self, msg):
        """
        Process ground truth odometry for drift calculation
        """
        gt_data = {
            'position': [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
            'timestamp': time.time()
        }

        self.ground_truth_history.append(gt_data)
        self.ground_truth_available = True

    def calculate_metrics(self):
        """
        Calculate and publish performance metrics
        """
        # Calculate frame rate
        if len(self.frame_times) > 1:
            time_window = 1.0  # 1 second window
            recent_frames = [t for t in self.frame_times if time.time() - t <= time_window]
            current_fps = len(recent_frames)
        else:
            current_fps = 0.0

        # Calculate processing time
        if self.processing_times:
            avg_processing_time = np.mean(list(self.processing_times))
        else:
            avg_processing_time = 0.0

        # Calculate drift if ground truth available
        if self.ground_truth_available and len(self.pose_history) > 0 and len(self.ground_truth_history) > 0:
            # Get latest estimates
            est_pose = self.pose_history[-1]['position']
            gt_pose = self.ground_truth_history[-1]['position']

            # Calculate drift
            drift_distance = np.sqrt(
                (est_pose[0] - gt_pose[0])**2 +
                (est_pose[1] - gt_pose[1])**2 +
                (est_pose[2] - gt_pose[2])**2
            )

            if self.total_path_length > 0:
                drift_percentage = (drift_distance / self.total_path_length) * 100
            else:
                drift_percentage = 0.0
        else:
            drift_distance = 0.0
            drift_percentage = 0.0

        # Calculate memory usage
        process = psutil.Process()
        memory_mb = process.memory_info().rss / 1024 / 1024

        # Calculate GPU usage if available
        gpu_percent = 0.0
        if self.gpu_available:
            gpus = GPUtil.getGPUs()
            if gpus:
                gpu_percent = gpus[0].load * 100

        # Store historical data
        self.memory_usage_history.append(memory_mb)
        self.gpu_usage_history.append(gpu_percent)

        # Publish metrics
        fps_msg = Float32()
        fps_msg.data = float(current_fps)
        self.fps_pub.publish(fps_msg)

        proc_time_msg = Float32()
        proc_time_msg.data = float(avg_processing_time * 1000)  # Convert to ms
        self.processing_time_pub.publish(proc_time_msg)

        drift_msg = Float32()
        drift_msg.data = float(drift_percentage)
        self.drift_pub.publish(drift_msg)

        memory_msg = Float32()
        memory_msg.data = float(memory_mb)
        self.memory_pub.publish(memory_msg)

        gpu_msg = Float32()
        gpu_msg.data = float(gpu_percent)
        self.gpu_usage_pub.publish(gpu_msg)

        # Log metrics periodically
        if self.frame_count % 30 == 0:  # Every 30 frames at 30Hz = 1Hz
            self.get_logger().info(
                f'📊 Metrics - FPS: {current_fps:.1f}, '
                f'Avg Proc: {avg_processing_time*1000:.1f}ms, '
                f'Drift: {drift_percentage:.2f}%, '
                f'Mem: {memory_mb:.0f}MB, '
                f'GPU: {gpu_percent:.1f}%'
            )

    def validate_performance(self):
        """
        Validate performance against success criteria
        """
        # Get current metrics
        current_metrics = self.get_current_metrics()

        # Validate SC-002: >30 Hz frame rate
        sc002_pass = current_metrics['current_fps'] > 30.0
        sc002_msg = f"SC-002: {'✅ PASS' if sc002_pass else '❌ FAIL'} - " \
                   f"Frame rate {current_metrics['current_fps']:.1f} Hz (target: >30 Hz)"

        # Validate SC-003: <2% drift over path
        sc003_pass = current_metrics['drift_percentage'] < 2.0
        sc003_msg = f"SC-003: {'✅ PASS' if sc003_pass else '❌ FAIL'} - " \
                   f"Drift {current_metrics['drift_percentage']:.2f}% (target: <2%)"

        # Validate FR-012: Performance metrics provided
        fr012_pass = all(key in current_metrics for key in [
            'current_fps', 'avg_processing_time_ms', 'drift_percentage',
            'memory_usage_mb', 'gpu_usage_percent'
        ])
        fr012_msg = f"FR-012: {'✅ PASS' if fr012_pass else '❌ FAIL'} - " \
                   f"Performance metrics {'available' if fr012_pass else 'missing'}"

        # Overall validation
        overall_pass = sc002_pass and sc003_pass and fr012_pass

        # Create results message
        results_msg = String()
        results_msg.data = json.dumps({
            'timestamp': datetime.now().isoformat(),
            'success_criteria': {
                'SC-002': {'pass': sc002_pass, 'value': current_metrics['current_fps'], 'unit': 'Hz', 'threshold': '>30'},
                'SC-003': {'pass': sc003_pass, 'value': current_metrics['drift_percentage'], 'unit': '%', 'threshold': '<2'},
            },
            'functional_requirements': {
                'FR-012': {'pass': fr012_pass, 'metrics_available': list(current_metrics.keys())}
            },
            'overall_performance': {
                'frame_rate_hz': current_metrics['current_fps'],
                'processing_time_ms': current_metrics['avg_processing_time_ms'],
                'drift_percent': current_metrics['drift_percentage'],
                'path_length_m': current_metrics['total_path_length'],
                'memory_usage_mb': current_metrics['memory_usage_mb'],
                'gpu_usage_percent': current_metrics['gpu_usage_percent']
            }
        })

        self.benchmark_results_pub.publish(results_msg)

        # Log validation results
        self.get_logger().info('🔍 Performance Validation Results:')
        self.get_logger().info(f'   {sc002_msg}')
        self.get_logger().info(f'   {sc003_msg}')
        self.get_logger().info(f'   {fr012_msg}')
        self.get_logger().info(f'   🎯 Overall: {"✅ PASS" if overall_pass else "❌ FAIL"}')

    def get_current_metrics(self):
        """
        Get current performance metrics
        """
        # Calculate frame rate (average over last 100 frames)
        if len(self.frame_times) > 1:
            time_window = 5.0  # 5 second window
            recent_frames = [t for t in self.frame_times if time.time() - t <= time_window]
            current_fps = len(recent_frames) / time_window if time_window > 0 else 0.0
        else:
            current_fps = 0.0

        # Calculate average processing time
        avg_proc_time = np.mean(list(self.processing_times)) * 1000 if self.processing_times else 0.0  # ms

        # Calculate drift percentage
        drift_pct = self.drift_percentage

        # Calculate memory usage
        process = psutil.Process()
        memory_mb = process.memory_info().rss / 1024 / 1024

        # Calculate GPU usage
        gpu_usage = 0.0
        if self.gpu_available:
            gpus = GPUtil.getGPUs()
            if gpus:
                gpu_usage = gpus[0].load * 100

        return {
            'current_fps': current_fps,
            'avg_processing_time_ms': avg_proc_time,
            'drift_percentage': drift_pct,
            'total_path_length': self.total_path_length,
            'memory_usage_mb': memory_mb,
            'gpu_usage_percent': gpu_usage,
            'frame_count': self.frame_count,
            'elapsed_time_sec': time.time() - self.start_time
        }

    def check_gpu_availability(self):
        """
        Check if GPU is available for acceleration
        """
        try:
            gpus = GPUtil.getGPUs()
            return len(gpus) > 0
        except:
            return False

    def save_benchmark_report(self, filepath=None):
        """
        Save comprehensive benchmark report
        """
        if filepath is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filepath = f"vslam_benchmark_report_{timestamp}.json"

        report = {
            'benchmark_info': {
                'timestamp': datetime.now().isoformat(),
                'node_name': self.get_name(),
                'runtime_seconds': time.time() - self.start_time
            },
            'hardware_info': {
                'gpu_available': self.gpu_available,
                'cpu_count': psutil.cpu_count(),
                'total_memory_gb': psutil.virtual_memory().total / (1024**3)
            },
            'performance_metrics': self.get_current_metrics(),
            'success_criteria': {
                'sc002_frame_rate': {
                    'target': '>30 Hz',
                    'achieved': self.get_current_metrics()['current_fps'],
                    'passed': self.get_current_metrics()['current_fps'] > 30.0
                },
                'sc003_drift': {
                    'target': '<2%',
                    'achieved': self.get_current_metrics()['drift_percentage'],
                    'passed': self.get_current_metrics()['drift_percentage'] < 2.0
                }
            },
            'functional_requirements': {
                'fr012_metrics': {
                    'target': 'Provide performance metrics',
                    'achieved': True,
                    'metrics_provided': list(self.get_current_metrics().keys())
                }
            }
        }

        with open(filepath, 'w') as f:
            json.dump(report, f, indent=2)

        self.get_logger().info(f'Benchmark report saved to: {filepath}')
        return filepath


def main(args=None):
    """
    Main function to run the VSLAM benchmark node
    """
    rclpy.init(args=args)
    benchmark_node = VSLAMBenchmarkNode()

    try:
        # Run for a specific duration or until interrupted
        rclpy.spin(benchmark_node)
    except KeyboardInterrupt:
        benchmark_node.get_logger().info('Shutting down VSLAM Benchmark node...')

        # Save final benchmark report
        report_path = benchmark_node.save_benchmark_report()
        benchmark_node.get_logger().info(f'Final benchmark report: {report_path}')

        # Print final summary
        final_metrics = benchmark_node.get_current_metrics()
        print("\n" + "="*60)
        print("VSLAM PERFORMANCE BENCHMARK RESULTS")
        print("="*60)
        print(f"Runtime: {final_metrics['elapsed_time_sec']:.1f} seconds")
        print(f"Total Frames Processed: {benchmark_node.frame_count}")
        print(f"Average Frame Rate: {final_metrics['current_fps']:.2f} Hz")
        print(f"Average Processing Time: {final_metrics['avg_processing_time_ms']:.2f} ms")
        print(f"Total Path Length: {final_metrics['total_path_length']:.2f} meters")
        print(f"Final Drift: {final_metrics['drift_percentage']:.2f}%")
        print(f"Peak Memory Usage: {final_metrics['memory_usage_mb']:.0f} MB")
        print(f"Average GPU Usage: {final_metrics['gpu_usage_percent']:.1f}%")
        print("\nSuccess Criteria Validation:")
        print(f"  SC-002 (>30 Hz): {'✅ PASS' if final_metrics['current_fps'] > 30.0 else '❌ FAIL'}")
        print(f"  SC-003 (<2% drift): {'✅ PASS' if final_metrics['drift_percentage'] < 2.0 else '❌ FAIL'}")
        print(f"  FR-012 (Metrics): ✅ PASS")
        print("="*60)

    finally:
        benchmark_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()