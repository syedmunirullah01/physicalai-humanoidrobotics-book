"""
Performance Metrics Utility for Module 3: The AI-Robot Brain (NVIDIA Isaac™)

Provides performance measurement tools for:
- VSLAM frame rate and latency
- Path planning computation time
- Synthetic data generation throughput
- GPU utilization

Usage:
    from shared.utils.metrics import VSLAMMetrics, NavigationMetrics, GPUMetrics

    # Measure VSLAM performance
    vslam_metrics = VSLAMMetrics()
    vslam_metrics.start()
    # ... run VSLAM ...
    vslam_metrics.record_frame(pose, num_features)
    stats = vslam_metrics.get_statistics()
"""

import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Tuple

import psutil

try:
    import GPUtil
    GPU_AVAILABLE = True
except ImportError:
    GPU_AVAILABLE = False


@dataclass
class VSLAMMetrics:
    """Metrics for Visual SLAM performance"""

    # Configuration
    window_size: int = 100  # Number of frames to track for moving averages

    # Internal state
    _start_time: float = field(default_factory=time.time, init=False)
    _frame_times: Deque[float] = field(default_factory=lambda: deque(maxlen=100), init=False)
    _frame_features: Deque[int] = field(default_factory=lambda: deque(maxlen=100), init=False)
    _total_frames: int = field(default=0, init=False)
    _tracking_failures: int = field(default=0, init=False)

    def start(self):
        """Start or restart metrics collection"""
        self._start_time = time.time()
        self._frame_times.clear()
        self._frame_features.clear()
        self._total_frames = 0
        self._tracking_failures = 0

    def record_frame(self, pose: Optional[Tuple[float, float, float]] = None, num_features: int = 0):
        """
        Record a VSLAM frame.

        Args:
            pose: Camera pose (x, y, z) or None if tracking failed
            num_features: Number of features detected/tracked
        """
        current_time = time.time()
        self._frame_times.append(current_time)
        self._frame_features.append(num_features)
        self._total_frames += 1

        if pose is None:
            self._tracking_failures += 1

    def get_statistics(self) -> Dict[str, float]:
        """
        Get VSLAM performance statistics.

        Returns:
            Dictionary with metrics:
            - frame_rate_hz: Instantaneous frame rate
            - avg_frame_rate_hz: Average frame rate over window
            - latency_ms: Average latency per frame
            - avg_features: Average features per frame
            - tracking_success_rate: % of frames with successful tracking
            - total_frames: Total frames processed
        """
        if len(self._frame_times) < 2:
            return {
                'frame_rate_hz': 0.0,
                'avg_frame_rate_hz': 0.0,
                'latency_ms': 0.0,
                'avg_features': 0.0,
                'tracking_success_rate': 0.0,
                'total_frames': self._total_frames
            }

        # Calculate frame rate
        time_diffs = [self._frame_times[i] - self._frame_times[i-1]
                     for i in range(1, len(self._frame_times))]
        avg_time_diff = sum(time_diffs) / len(time_diffs)
        instantaneous_fps = 1.0 / time_diffs[-1] if time_diffs else 0.0
        avg_fps = 1.0 / avg_time_diff if avg_time_diff > 0 else 0.0

        # Calculate latency
        latency_ms = avg_time_diff * 1000

        # Calculate average features
        avg_features = sum(self._frame_features) / len(self._frame_features)

        # Calculate tracking success rate
        successful_frames = self._total_frames - self._tracking_failures
        success_rate = (successful_frames / self._total_frames * 100) if self._total_frames > 0 else 0.0

        return {
            'frame_rate_hz': instantaneous_fps,
            'avg_frame_rate_hz': avg_fps,
            'latency_ms': latency_ms,
            'avg_features': avg_features,
            'tracking_success_rate': success_rate,
            'total_frames': self._total_frames
        }

    def meets_success_criteria(self) -> Tuple[bool, str]:
        """
        Check if VSLAM performance meets Module 3 success criteria.

        Returns:
            Tuple of (meets_criteria, message)

        Success Criteria:
        - SC-002: >30 Hz frame rate
        - Tracking success rate >90%
        """
        stats = self.get_statistics()

        if stats['avg_frame_rate_hz'] < 30:
            return False, f"Frame rate {stats['avg_frame_rate_hz']:.1f} Hz < 30 Hz (SC-002)"

        if stats['tracking_success_rate'] < 90:
            return False, f"Tracking success {stats['tracking_success_rate']:.1f}% < 90%"

        return True, f"VSLAM performance OK: {stats['avg_frame_rate_hz']:.1f} Hz, {stats['tracking_success_rate']:.1f}% success"


@dataclass
class NavigationMetrics:
    """Metrics for Nav2 navigation performance"""

    # Internal state
    _goals: List[Dict] = field(default_factory=list, init=False)
    _current_goal_start: Optional[float] = field(default=None, init=False)

    def start_goal(self):
        """Record start of navigation goal"""
        self._current_goal_start = time.time()

    def end_goal(self, success: bool, distance_traveled: float = 0.0, num_replans: int = 0):
        """
        Record completion of navigation goal.

        Args:
            success: Whether goal was reached successfully
            distance_traveled: Total distance traveled (meters)
            num_replans: Number of times path was replanned
        """
        if self._current_goal_start is None:
            return

        elapsed_time = time.time() - self._current_goal_start

        self._goals.append({
            'success': success,
            'time_sec': elapsed_time,
            'distance_m': distance_traveled,
            'replans': num_replans
        })

        self._current_goal_start = None

    def record_planning_time(self, planning_time_sec: float):
        """Record time taken for path planning"""
        if self._goals:
            self._goals[-1]['planning_time_sec'] = planning_time_sec

    def get_statistics(self) -> Dict[str, float]:
        """
        Get navigation performance statistics.

        Returns:
            Dictionary with metrics:
            - success_rate: % of goals reached successfully
            - avg_planning_time_sec: Average time to compute path
            - avg_execution_time_sec: Average time to reach goal
            - avg_replans: Average number of replans per goal
            - total_goals: Total goals attempted
        """
        if not self._goals:
            return {
                'success_rate': 0.0,
                'avg_planning_time_sec': 0.0,
                'avg_execution_time_sec': 0.0,
                'avg_replans': 0.0,
                'total_goals': 0
            }

        successful_goals = [g for g in self._goals if g['success']]
        success_rate = len(successful_goals) / len(self._goals) * 100

        planning_times = [g.get('planning_time_sec', 0) for g in self._goals if 'planning_time_sec' in g]
        avg_planning_time = sum(planning_times) / len(planning_times) if planning_times else 0.0

        execution_times = [g['time_sec'] for g in self._goals]
        avg_execution_time = sum(execution_times) / len(execution_times)

        replans = [g['replans'] for g in self._goals]
        avg_replans = sum(replans) / len(replans)

        return {
            'success_rate': success_rate,
            'avg_planning_time_sec': avg_planning_time,
            'avg_execution_time_sec': avg_execution_time,
            'avg_replans': avg_replans,
            'total_goals': len(self._goals)
        }

    def meets_success_criteria(self) -> Tuple[bool, str]:
        """
        Check if navigation performance meets Module 3 success criteria.

        Returns:
            Tuple of (meets_criteria, message)

        Success Criteria:
        - SC-004: Planning time <5 seconds
        - SC-008: Success rate >95%
        """
        stats = self.get_statistics()

        if stats['avg_planning_time_sec'] > 5.0:
            return False, f"Planning time {stats['avg_planning_time_sec']:.2f}s > 5s (SC-004)"

        if stats['success_rate'] < 95:
            return False, f"Success rate {stats['success_rate']:.1f}% < 95% (SC-008)"

        return True, f"Navigation performance OK: {stats['avg_planning_time_sec']:.2f}s planning, {stats['success_rate']:.1f}% success"


@dataclass
class DatasetMetrics:
    """Metrics for synthetic dataset generation"""

    # Internal state
    _start_time: Optional[float] = field(default=None, init=False)
    _images_generated: int = field(default=0, init=False)

    def start(self):
        """Start dataset generation timing"""
        self._start_time = time.time()
        self._images_generated = 0

    def record_image(self):
        """Record generation of one image"""
        self._images_generated += 1

    def get_statistics(self) -> Dict[str, float]:
        """
        Get dataset generation statistics.

        Returns:
            Dictionary with metrics:
            - images_per_hour: Generation rate
            - elapsed_time_sec: Total time elapsed
            - total_images: Total images generated
        """
        if self._start_time is None:
            return {
                'images_per_hour': 0.0,
                'elapsed_time_sec': 0.0,
                'total_images': 0
            }

        elapsed_sec = time.time() - self._start_time
        images_per_hour = (self._images_generated / elapsed_sec * 3600) if elapsed_sec > 0 else 0.0

        return {
            'images_per_hour': images_per_hour,
            'elapsed_time_sec': elapsed_sec,
            'total_images': self._images_generated
        }

    def meets_success_criteria(self) -> Tuple[bool, str]:
        """
        Check if dataset generation meets Module 3 success criteria.

        Returns:
            Tuple of (meets_criteria, message)

        Success Criteria:
        - SC-001: 1000+ images per hour
        """
        stats = self.get_statistics()

        if stats['images_per_hour'] < 1000:
            return False, f"Generation rate {stats['images_per_hour']:.0f} images/hour < 1000 (SC-001)"

        return True, f"Dataset generation OK: {stats['images_per_hour']:.0f} images/hour"


class GPUMetrics:
    """GPU utilization and memory metrics"""

    @staticmethod
    def get_gpu_stats() -> Optional[Dict[str, float]]:
        """
        Get current GPU statistics.

        Returns:
            Dictionary with GPU metrics or None if GPU not available:
            - gpu_utilization_percent: GPU core utilization
            - memory_used_mb: VRAM used
            - memory_total_mb: Total VRAM
            - memory_utilization_percent: VRAM utilization
            - temperature_c: GPU temperature
        """
        if not GPU_AVAILABLE:
            return None

        try:
            gpus = GPUtil.getGPUs()
            if not gpus:
                return None

            gpu = gpus[0]  # Use first GPU

            return {
                'gpu_utilization_percent': gpu.load * 100,
                'memory_used_mb': gpu.memoryUsed,
                'memory_total_mb': gpu.memoryTotal,
                'memory_utilization_percent': gpu.memoryUtil * 100,
                'temperature_c': gpu.temperature
            }
        except Exception:
            return None

    @staticmethod
    def get_system_stats() -> Dict[str, float]:
        """
        Get system-wide performance statistics.

        Returns:
            Dictionary with system metrics:
            - cpu_utilization_percent: CPU utilization
            - ram_used_mb: RAM used
            - ram_total_mb: Total RAM
            - ram_utilization_percent: RAM utilization
        """
        memory = psutil.virtual_memory()

        return {
            'cpu_utilization_percent': psutil.cpu_percent(interval=0.1),
            'ram_used_mb': memory.used / (1024 * 1024),
            'ram_total_mb': memory.total / (1024 * 1024),
            'ram_utilization_percent': memory.percent
        }


def print_metrics_report(vslam: Optional[VSLAMMetrics] = None,
                        navigation: Optional[NavigationMetrics] = None,
                        dataset: Optional[DatasetMetrics] = None):
    """
    Print formatted metrics report for all tracked metrics.

    Args:
        vslam: VSLAM metrics (optional)
        navigation: Navigation metrics (optional)
        dataset: Dataset generation metrics (optional)
    """
    print("=" * 60)
    print("Module 3: Performance Metrics Report")
    print("=" * 60)

    # VSLAM metrics
    if vslam:
        stats = vslam.get_statistics()
        meets_criteria, msg = vslam.meets_success_criteria()

        print("\n📊 VSLAM Performance:")
        print(f"  Frame Rate: {stats['avg_frame_rate_hz']:.1f} Hz (target: >30 Hz)")
        print(f"  Latency: {stats['latency_ms']:.1f} ms")
        print(f"  Features: {stats['avg_features']:.0f} avg")
        print(f"  Tracking Success: {stats['tracking_success_rate']:.1f}%")
        print(f"  Total Frames: {stats['total_frames']}")
        print(f"  Status: {'✅ PASS' if meets_criteria else '❌ FAIL'} - {msg}")

    # Navigation metrics
    if navigation:
        stats = navigation.get_statistics()
        meets_criteria, msg = navigation.meets_success_criteria()

        print("\n🗺️  Navigation Performance:")
        print(f"  Planning Time: {stats['avg_planning_time_sec']:.2f}s (target: <5s)")
        print(f"  Execution Time: {stats['avg_execution_time_sec']:.2f}s avg")
        print(f"  Success Rate: {stats['success_rate']:.1f}% (target: >95%)")
        print(f"  Avg Replans: {stats['avg_replans']:.1f}")
        print(f"  Total Goals: {stats['total_goals']}")
        print(f"  Status: {'✅ PASS' if meets_criteria else '❌ FAIL'} - {msg}")

    # Dataset metrics
    if dataset:
        stats = dataset.get_statistics()
        meets_criteria, msg = dataset.meets_success_criteria()

        print("\n🖼️  Dataset Generation:")
        print(f"  Generation Rate: {stats['images_per_hour']:.0f} images/hour (target: >1000)")
        print(f"  Elapsed Time: {stats['elapsed_time_sec']:.1f}s")
        print(f"  Total Images: {stats['total_images']}")
        print(f"  Status: {'✅ PASS' if meets_criteria else '❌ FAIL'} - {msg}")

    # GPU metrics
    gpu_stats = GPUMetrics.get_gpu_stats()
    if gpu_stats:
        print("\n🎮 GPU Utilization:")
        print(f"  GPU Load: {gpu_stats['gpu_utilization_percent']:.1f}%")
        print(f"  VRAM: {gpu_stats['memory_used_mb']:.0f} / {gpu_stats['memory_total_mb']:.0f} MB ({gpu_stats['memory_utilization_percent']:.1f}%)")
        print(f"  Temperature: {gpu_stats['temperature_c']:.1f}°C")

    # System metrics
    sys_stats = GPUMetrics.get_system_stats()
    print("\n💻 System Resources:")
    print(f"  CPU: {sys_stats['cpu_utilization_percent']:.1f}%")
    print(f"  RAM: {sys_stats['ram_used_mb']:.0f} / {sys_stats['ram_total_mb']:.0f} MB ({sys_stats['ram_utilization_percent']:.1f}%)")

    print("\n" + "=" * 60)
