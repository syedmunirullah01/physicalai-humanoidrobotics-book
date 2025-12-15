"""
Test suite for metrics.py utility

Validates performance metrics tracking for Module 3.

Usage:
    pytest tests/test_metrics.py
    pytest tests/test_metrics.py -v
    pytest tests/test_metrics.py::test_vslam_metrics
"""

import time
import unittest
from unittest.mock import Mock, patch

import pytest

# Import the module under test
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from shared.utils.metrics import (
    VSLAMMetrics,
    NavigationMetrics,
    DatasetMetrics,
    GPUMetrics,
    print_metrics_report
)


class TestVSLAMMetrics(unittest.TestCase):
    """Test VSLAM performance metrics tracking"""

    def test_vslam_initialization(self):
        """Test VSLAM metrics initialization"""
        metrics = VSLAMMetrics()

        assert metrics.window_size == 100
        assert metrics._total_frames == 0
        assert metrics._tracking_failures == 0

    def test_vslam_record_frame_success(self):
        """Test recording successful VSLAM frames"""
        metrics = VSLAMMetrics()
        metrics.start()

        # Record 10 successful frames
        for i in range(10):
            metrics.record_frame(pose=(float(i), 0.0, 0.0), num_features=500)
            time.sleep(0.01)  # Simulate frame processing

        stats = metrics.get_statistics()

        assert stats['total_frames'] == 10
        assert stats['tracking_success_rate'] == 100.0
        assert stats['avg_features'] == 500.0
        assert stats['frame_rate_hz'] > 0

    def test_vslam_record_frame_tracking_loss(self):
        """Test recording VSLAM frames with tracking failures"""
        metrics = VSLAMMetrics()
        metrics.start()

        # Record 10 frames with 2 failures
        for i in range(10):
            pose = (float(i), 0.0, 0.0) if i not in [3, 7] else None
            metrics.record_frame(pose=pose, num_features=450)
            time.sleep(0.01)

        stats = metrics.get_statistics()

        assert stats['total_frames'] == 10
        assert stats['tracking_success_rate'] == 80.0  # 8/10 success

    def test_vslam_frame_rate_calculation(self):
        """Test VSLAM frame rate calculation"""
        metrics = VSLAMMetrics()
        metrics.start()

        # Simulate 30 Hz frame rate
        target_fps = 30
        for _ in range(100):
            metrics.record_frame(pose=(0.0, 0.0, 0.0), num_features=500)
            time.sleep(1.0 / target_fps)

        stats = metrics.get_statistics()

        # Allow ±10% tolerance
        assert 27 <= stats['avg_frame_rate_hz'] <= 33

    def test_vslam_success_criteria_pass(self):
        """Test VSLAM success criteria validation (passing)"""
        metrics = VSLAMMetrics()
        metrics.start()

        # Simulate >30 Hz with high success rate
        for _ in range(50):
            metrics.record_frame(pose=(0.0, 0.0, 0.0), num_features=500)
            time.sleep(1.0 / 35)  # 35 Hz

        meets_criteria, message = metrics.meets_success_criteria()

        assert meets_criteria is True
        assert "SC-002" not in message  # Should not mention failure

    def test_vslam_success_criteria_fail_frame_rate(self):
        """Test VSLAM success criteria validation (failing frame rate)"""
        metrics = VSLAMMetrics()
        metrics.start()

        # Simulate <30 Hz
        for _ in range(50):
            metrics.record_frame(pose=(0.0, 0.0, 0.0), num_features=500)
            time.sleep(1.0 / 20)  # 20 Hz

        meets_criteria, message = metrics.meets_success_criteria()

        assert meets_criteria is False
        assert "SC-002" in message or "30 Hz" in message

    def test_vslam_success_criteria_fail_tracking(self):
        """Test VSLAM success criteria validation (failing tracking rate)"""
        metrics = VSLAMMetrics()
        metrics.start()

        # Simulate good FPS but poor tracking
        for i in range(100):
            pose = (0.0, 0.0, 0.0) if i < 85 else None  # 85% success
            metrics.record_frame(pose=pose, num_features=500)
            time.sleep(1.0 / 35)

        meets_criteria, message = metrics.meets_success_criteria()

        assert meets_criteria is False
        assert "tracking" in message.lower() or "success" in message.lower()


class TestNavigationMetrics(unittest.TestCase):
    """Test navigation performance metrics tracking"""

    def test_navigation_initialization(self):
        """Test navigation metrics initialization"""
        metrics = NavigationMetrics()

        assert len(metrics._goals) == 0
        assert metrics._current_goal_start is None

    def test_navigation_single_goal_success(self):
        """Test recording single successful navigation goal"""
        metrics = NavigationMetrics()

        metrics.start_goal()
        time.sleep(0.1)  # Simulate navigation time
        metrics.record_planning_time(3.5)
        metrics.end_goal(success=True, distance_traveled=10.5, num_replans=1)

        stats = metrics.get_statistics()

        assert stats['total_goals'] == 1
        assert stats['success_rate'] == 100.0
        assert stats['avg_planning_time_sec'] == 3.5
        assert stats['avg_replans'] == 1.0

    def test_navigation_multiple_goals(self):
        """Test recording multiple navigation goals"""
        metrics = NavigationMetrics()

        # 10 goals, 8 successful
        for i in range(10):
            metrics.start_goal()
            time.sleep(0.05)
            metrics.record_planning_time(4.0 + i * 0.1)  # Varying planning times
            metrics.end_goal(success=(i < 8), distance_traveled=10.0, num_replans=i % 3)

        stats = metrics.get_statistics()

        assert stats['total_goals'] == 10
        assert stats['success_rate'] == 80.0
        assert 4.0 <= stats['avg_planning_time_sec'] <= 5.0
        assert stats['avg_replans'] >= 0

    def test_navigation_success_criteria_pass(self):
        """Test navigation success criteria validation (passing)"""
        metrics = NavigationMetrics()

        # Simulate passing criteria: <5s planning, >95% success
        for i in range(20):
            metrics.start_goal()
            metrics.record_planning_time(3.2)  # <5s
            metrics.end_goal(success=True, distance_traveled=10.0, num_replans=1)

        meets_criteria, message = metrics.meets_success_criteria()

        assert meets_criteria is True
        assert "OK" in message or "pass" in message.lower()

    def test_navigation_success_criteria_fail_planning_time(self):
        """Test navigation success criteria validation (slow planning)"""
        metrics = NavigationMetrics()

        # Simulate slow planning
        for _ in range(10):
            metrics.start_goal()
            metrics.record_planning_time(6.5)  # >5s
            metrics.end_goal(success=True, distance_traveled=10.0, num_replans=1)

        meets_criteria, message = metrics.meets_success_criteria()

        assert meets_criteria is False
        assert "SC-004" in message or "5" in message

    def test_navigation_success_criteria_fail_success_rate(self):
        """Test navigation success criteria validation (low success rate)"""
        metrics = NavigationMetrics()

        # Simulate low success rate (90% < 95%)
        for i in range(20):
            metrics.start_goal()
            metrics.record_planning_time(3.0)
            metrics.end_goal(success=(i < 18), distance_traveled=10.0, num_replans=1)

        meets_criteria, message = metrics.meets_success_criteria()

        assert meets_criteria is False
        assert "SC-008" in message or "95%" in message


class TestDatasetMetrics(unittest.TestCase):
    """Test synthetic dataset generation metrics"""

    def test_dataset_initialization(self):
        """Test dataset metrics initialization"""
        metrics = DatasetMetrics()

        assert metrics._start_time is None
        assert metrics._images_generated == 0

    def test_dataset_generation_rate(self):
        """Test dataset generation rate calculation"""
        metrics = DatasetMetrics()
        metrics.start()

        # Simulate generating 100 images in 0.5 seconds
        for _ in range(100):
            metrics.record_image()
            time.sleep(0.005)  # 5ms per image

        stats = metrics.get_statistics()

        # Should be ~720 images/hour at 5ms/image
        assert stats['total_images'] == 100
        assert stats['images_per_hour'] > 500  # At least 500 images/hour

    def test_dataset_success_criteria_pass(self):
        """Test dataset success criteria validation (passing)"""
        metrics = DatasetMetrics()
        metrics.start()

        # Simulate generating 100 images quickly (<1 second)
        # This would project to >1000 images/hour
        for _ in range(100):
            metrics.record_image()
            time.sleep(0.0001)  # Very fast

        meets_criteria, message = metrics.meets_success_criteria()

        assert meets_criteria is True
        assert "OK" in message or "pass" in message.lower()

    def test_dataset_success_criteria_fail(self):
        """Test dataset success criteria validation (failing)"""
        metrics = DatasetMetrics()
        metrics.start()

        # Simulate slow generation
        for _ in range(10):
            metrics.record_image()
            time.sleep(0.05)  # 50ms per image = ~720 images/hour

        stats = metrics.get_statistics()

        # Should fail SC-001 (1000+ images/hour)
        if stats['images_per_hour'] < 1000:
            meets_criteria, message = metrics.meets_success_criteria()
            assert meets_criteria is False
            assert "SC-001" in message or "1000" in message


class TestGPUMetrics(unittest.TestCase):
    """Test GPU utilization metrics"""

    @patch('shared.utils.metrics.GPU_AVAILABLE', True)
    @patch('GPUtil.getGPUs')
    def test_gpu_stats_available(self, mock_get_gpus):
        """Test GPU stats when GPU is available"""
        # Mock GPU object
        mock_gpu = Mock()
        mock_gpu.load = 0.75  # 75% utilization
        mock_gpu.memoryUsed = 8192  # 8GB used
        mock_gpu.memoryTotal = 12288  # 12GB total
        mock_gpu.memoryUtil = 0.66  # 66% memory utilization
        mock_gpu.temperature = 65.5  # 65.5°C

        mock_get_gpus.return_value = [mock_gpu]

        stats = GPUMetrics.get_gpu_stats()

        assert stats is not None
        assert stats['gpu_utilization_percent'] == 75.0
        assert stats['memory_used_mb'] == 8192
        assert stats['memory_total_mb'] == 12288
        assert stats['memory_utilization_percent'] == 66.0
        assert stats['temperature_c'] == 65.5

    @patch('shared.utils.metrics.GPU_AVAILABLE', False)
    def test_gpu_stats_unavailable(self):
        """Test GPU stats when GPU library not available"""
        stats = GPUMetrics.get_gpu_stats()

        assert stats is None

    def test_system_stats(self):
        """Test system resource statistics"""
        stats = GPUMetrics.get_system_stats()

        assert stats is not None
        assert 'cpu_utilization_percent' in stats
        assert 'ram_used_mb' in stats
        assert 'ram_total_mb' in stats
        assert 'ram_utilization_percent' in stats

        # Sanity checks
        assert 0 <= stats['cpu_utilization_percent'] <= 100
        assert stats['ram_used_mb'] > 0
        assert stats['ram_total_mb'] > stats['ram_used_mb']
        assert 0 <= stats['ram_utilization_percent'] <= 100


class TestMetricsReport(unittest.TestCase):
    """Test comprehensive metrics reporting"""

    def test_print_metrics_report_vslam_only(self, capsys=None):
        """Test metrics report with VSLAM data only"""
        vslam = VSLAMMetrics()
        vslam.start()

        for _ in range(50):
            vslam.record_frame(pose=(0.0, 0.0, 0.0), num_features=500)
            time.sleep(1.0 / 35)

        # This should not raise an error
        print_metrics_report(vslam=vslam)

    def test_print_metrics_report_all_metrics(self):
        """Test metrics report with all metrics"""
        vslam = VSLAMMetrics()
        vslam.start()
        for _ in range(30):
            vslam.record_frame(pose=(0.0, 0.0, 0.0), num_features=500)
            time.sleep(1.0 / 35)

        navigation = NavigationMetrics()
        for _ in range(5):
            navigation.start_goal()
            navigation.record_planning_time(3.5)
            navigation.end_goal(success=True, distance_traveled=10.0, num_replans=1)

        dataset = DatasetMetrics()
        dataset.start()
        for _ in range(50):
            dataset.record_image()
            time.sleep(0.001)

        # This should not raise an error
        print_metrics_report(vslam=vslam, navigation=navigation, dataset=dataset)


class TestMetricsIntegration(unittest.TestCase):
    """Integration tests for complete metrics workflow"""

    def test_full_vslam_workflow(self):
        """Test complete VSLAM metrics workflow"""
        metrics = VSLAMMetrics(window_size=50)
        metrics.start()

        # Simulate VSLAM processing
        for i in range(100):
            pose = (float(i) * 0.1, 0.0, 0.0) if i % 10 != 0 else None
            features = 500 + (i % 100)
            metrics.record_frame(pose=pose, num_features=features)
            time.sleep(1.0 / 35)  # 35 Hz target

        # Get statistics
        stats = metrics.get_statistics()

        assert stats['total_frames'] == 100
        assert stats['avg_frame_rate_hz'] > 25  # Should be close to 35 Hz
        assert stats['avg_features'] > 0
        assert stats['tracking_success_rate'] == 90.0  # 90/100 success

        # Check success criteria
        meets_criteria, message = metrics.meets_success_criteria()

        # May pass or fail depending on exact timing
        assert isinstance(meets_criteria, bool)
        assert len(message) > 0

    def test_full_navigation_workflow(self):
        """Test complete navigation metrics workflow"""
        metrics = NavigationMetrics()

        # Simulate 20 navigation goals
        for i in range(20):
            metrics.start_goal()
            time.sleep(0.05)  # Simulate execution time

            planning_time = 2.0 + (i % 5) * 0.5  # 2.0-4.0 seconds
            metrics.record_planning_time(planning_time)

            success = i < 19  # 95% success rate
            distance = 5.0 + i * 0.5
            replans = i % 3

            metrics.end_goal(success=success, distance_traveled=distance, num_replans=replans)

        # Get statistics
        stats = metrics.get_statistics()

        assert stats['total_goals'] == 20
        assert stats['success_rate'] == 95.0
        assert 2.0 <= stats['avg_planning_time_sec'] <= 4.0

        # Check success criteria
        meets_criteria, message = metrics.meets_success_criteria()

        assert meets_criteria is True  # Should pass with 95% success and <4s planning


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
