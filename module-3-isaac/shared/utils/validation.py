"""
Success Criteria Validation Utility for Module 3: The AI-Robot Brain (NVIDIA Isaac™)

Validates Module 3 success criteria (SC-001 through SC-009) against actual performance.

Usage:
    from shared.utils.validation import validate_all_criteria, ValidationResult

    results = validate_all_criteria()
    print(f"Overall: {'PASS' if results.all_passed else 'FAIL'}")
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional

from .metrics import DatasetMetrics, NavigationMetrics, VSLAMMetrics


@dataclass
class ValidationResult:
    """Result of a single success criteria validation"""
    criteria_id: str
    description: str
    target: str
    actual: str
    passed: bool
    message: str


@dataclass
class OverallValidation:
    """Overall validation results for all success criteria"""
    results: List[ValidationResult] = field(default_factory=list)

    @property
    def all_passed(self) -> bool:
        """Check if all criteria passed"""
        return all(r.passed for r in self.results)

    @property
    def pass_count(self) -> int:
        """Count of passed criteria"""
        return sum(1 for r in self.results if r.passed)

    @property
    def fail_count(self) -> int:
        """Count of failed criteria"""
        return sum(1 for r in self.results if not r.passed)

    def get_summary(self) -> str:
        """Get formatted summary of validation results"""
        total = len(self.results)
        passed = self.pass_count
        failed = self.fail_count

        summary = f"Validation Summary: {passed}/{total} passed, {failed}/{total} failed\n"
        summary += "=" * 60 + "\n"

        for result in self.results:
            status = "✅ PASS" if result.passed else "❌ FAIL"
            summary += f"\n{result.criteria_id}: {result.description}\n"
            summary += f"  Target: {result.target}\n"
            summary += f"  Actual: {result.actual}\n"
            summary += f"  Status: {status} - {result.message}\n"

        summary += "\n" + "=" * 60 + "\n"
        summary += f"Overall: {'✅ ALL CRITERIA PASSED' if self.all_passed else '❌ SOME CRITERIA FAILED'}\n"
        summary += "=" * 60

        return summary


def validate_sc001_dataset_generation(dataset_metrics: DatasetMetrics) -> ValidationResult:
    """
    Validate SC-001: Learners can create simulation environments and generate 1,000+
    labeled images within 1 hour of training time

    Args:
        dataset_metrics: Dataset generation metrics

    Returns:
        ValidationResult for SC-001
    """
    stats = dataset_metrics.get_statistics()
    images_per_hour = stats['images_per_hour']

    passed = images_per_hour >= 1000
    message = f"Dataset generation rate is {'sufficient' if passed else 'insufficient'}"

    return ValidationResult(
        criteria_id="SC-001",
        description="Generate 1000+ labeled images within 1 hour",
        target="≥1000 images/hour",
        actual=f"{images_per_hour:.0f} images/hour",
        passed=passed,
        message=message
    )


def validate_sc002_vslam_framerate(vslam_metrics: VSLAMMetrics) -> ValidationResult:
    """
    Validate SC-002: VSLAM implementation achieves real-time performance (>30 Hz)
    on standard development hardware with GPU acceleration

    Args:
        vslam_metrics: VSLAM performance metrics

    Returns:
        ValidationResult for SC-002
    """
    stats = vslam_metrics.get_statistics()
    frame_rate = stats['avg_frame_rate_hz']

    passed = frame_rate > 30
    message = f"VSLAM frame rate is {'real-time' if passed else 'below real-time'}"

    return ValidationResult(
        criteria_id="SC-002",
        description="VSLAM achieves >30 Hz real-time performance",
        target=">30 Hz",
        actual=f"{frame_rate:.1f} Hz",
        passed=passed,
        message=message
    )


def validate_sc003_vslam_accuracy(trajectory_error_percent: float) -> ValidationResult:
    """
    Validate SC-003: VSLAM trajectory accuracy maintains <2% drift over 100-meter
    paths in simulation environments

    Args:
        trajectory_error_percent: Measured trajectory drift percentage

    Returns:
        ValidationResult for SC-003
    """
    passed = trajectory_error_percent < 2.0
    message = f"Trajectory drift is {'acceptable' if passed else 'too high'}"

    return ValidationResult(
        criteria_id="SC-003",
        description="VSLAM maintains <2% drift over 100m paths",
        target="<2% drift",
        actual=f"{trajectory_error_percent:.2f}% drift",
        passed=passed,
        message=message
    )


def validate_sc004_planning_time(navigation_metrics: NavigationMetrics) -> ValidationResult:
    """
    Validate SC-004: Path planning generates valid bipedal trajectories within
    5 seconds for typical navigation scenarios

    Args:
        navigation_metrics: Navigation performance metrics

    Returns:
        ValidationResult for SC-004
    """
    stats = navigation_metrics.get_statistics()
    planning_time = stats['avg_planning_time_sec']

    passed = planning_time < 5.0
    message = f"Planning time is {'acceptable' if passed else 'too slow'}"

    return ValidationResult(
        criteria_id="SC-004",
        description="Path planning within 5 seconds",
        target="<5 seconds",
        actual=f"{planning_time:.2f} seconds",
        passed=passed,
        message=message
    )


def validate_sc005_completion_rate(completion_rate_percent: float) -> ValidationResult:
    """
    Validate SC-005: 90% of learners successfully complete all three chapters and
    demonstrate end-to-end autonomous navigation in simulation

    Args:
        completion_rate_percent: Measured learner completion rate

    Returns:
        ValidationResult for SC-005
    """
    passed = completion_rate_percent >= 90.0
    message = f"Completion rate is {'sufficient' if passed else 'insufficient'}"

    return ValidationResult(
        criteria_id="SC-005",
        description="90% learner completion rate",
        target="≥90%",
        actual=f"{completion_rate_percent:.1f}%",
        passed=passed,
        message=message
    )


def validate_sc006_sim_to_real_understanding(quiz_score_percent: float) -> ValidationResult:
    """
    Validate SC-006: Learners understand the sim-to-real gap and can explain at
    least 3 techniques to mitigate it

    Args:
        quiz_score_percent: Assessment score on sim-to-real concepts

    Returns:
        ValidationResult for SC-006
    """
    passed = quiz_score_percent >= 75.0  # Can explain 3+ techniques
    message = f"Sim-to-real understanding is {'demonstrated' if passed else 'insufficient'}"

    return ValidationResult(
        criteria_id="SC-006",
        description="Understand sim-to-real gap mitigation",
        target="≥75% quiz score (3+ techniques)",
        actual=f"{quiz_score_percent:.1f}% quiz score",
        passed=passed,
        message=message
    )


def validate_sc007_perception_accuracy(perception_accuracy_percent: float) -> ValidationResult:
    """
    Validate SC-007: Generated synthetic datasets achieve >85% accuracy when used
    to train perception models tested on validation data

    Args:
        perception_accuracy_percent: Perception model accuracy on validation set

    Returns:
        ValidationResult for SC-007
    """
    passed = perception_accuracy_percent > 85.0
    message = f"Perception accuracy is {'sufficient' if passed else 'insufficient'}"

    return ValidationResult(
        criteria_id="SC-007",
        description="Synthetic data achieves >85% perception accuracy",
        target=">85%",
        actual=f"{perception_accuracy_percent:.1f}%",
        passed=passed,
        message=message
    )


def validate_sc008_navigation_success(navigation_metrics: NavigationMetrics) -> ValidationResult:
    """
    Validate SC-008: Navigation success rate exceeds 95% for reaching goals in
    obstacle-free environments

    Args:
        navigation_metrics: Navigation performance metrics

    Returns:
        ValidationResult for SC-008
    """
    stats = navigation_metrics.get_statistics()
    success_rate = stats['success_rate']

    passed = success_rate > 95.0
    message = f"Navigation success rate is {'sufficient' if passed else 'insufficient'}"

    return ValidationResult(
        criteria_id="SC-008",
        description="Navigation success rate >95%",
        target=">95%",
        actual=f"{success_rate:.1f}%",
        passed=passed,
        message=message
    )


def validate_sc009_recovery_success(recovery_success_percent: float) -> ValidationResult:
    """
    Validate SC-009: System demonstrates graceful failure handling with recovery
    success rate >70% when navigation encounters obstacles or localization failures

    Args:
        recovery_success_percent: Measured recovery success rate

    Returns:
        ValidationResult for SC-009
    """
    passed = recovery_success_percent > 70.0
    message = f"Recovery success rate is {'sufficient' if passed else 'insufficient'}"

    return ValidationResult(
        criteria_id="SC-009",
        description="Recovery success rate >70%",
        target=">70%",
        actual=f"{recovery_success_percent:.1f}%",
        passed=passed,
        message=message
    )


def validate_all_criteria(
    dataset_metrics: Optional[DatasetMetrics] = None,
    vslam_metrics: Optional[VSLAMMetrics] = None,
    navigation_metrics: Optional[NavigationMetrics] = None,
    trajectory_error_percent: float = 0.0,
    completion_rate_percent: float = 0.0,
    quiz_score_percent: float = 0.0,
    perception_accuracy_percent: float = 0.0,
    recovery_success_percent: float = 0.0
) -> OverallValidation:
    """
    Validate all Module 3 success criteria.

    Args:
        dataset_metrics: Dataset generation metrics (for SC-001)
        vslam_metrics: VSLAM performance metrics (for SC-002)
        navigation_metrics: Navigation metrics (for SC-004, SC-008)
        trajectory_error_percent: VSLAM trajectory drift (for SC-003)
        completion_rate_percent: Learner completion rate (for SC-005)
        quiz_score_percent: Sim-to-real quiz score (for SC-006)
        perception_accuracy_percent: Perception model accuracy (for SC-007)
        recovery_success_percent: Recovery success rate (for SC-009)

    Returns:
        OverallValidation with results for all criteria
    """
    validation = OverallValidation()

    # SC-001: Dataset generation
    if dataset_metrics:
        validation.results.append(validate_sc001_dataset_generation(dataset_metrics))

    # SC-002: VSLAM frame rate
    if vslam_metrics:
        validation.results.append(validate_sc002_vslam_framerate(vslam_metrics))

    # SC-003: VSLAM accuracy
    if trajectory_error_percent >= 0:
        validation.results.append(validate_sc003_vslam_accuracy(trajectory_error_percent))

    # SC-004: Planning time
    if navigation_metrics:
        validation.results.append(validate_sc004_planning_time(navigation_metrics))

    # SC-005: Completion rate
    if completion_rate_percent >= 0:
        validation.results.append(validate_sc005_completion_rate(completion_rate_percent))

    # SC-006: Sim-to-real understanding
    if quiz_score_percent >= 0:
        validation.results.append(validate_sc006_sim_to_real_understanding(quiz_score_percent))

    # SC-007: Perception accuracy
    if perception_accuracy_percent >= 0:
        validation.results.append(validate_sc007_perception_accuracy(perception_accuracy_percent))

    # SC-008: Navigation success
    if navigation_metrics:
        validation.results.append(validate_sc008_navigation_success(navigation_metrics))

    # SC-009: Recovery success
    if recovery_success_percent >= 0:
        validation.results.append(validate_sc009_recovery_success(recovery_success_percent))

    return validation


def main():
    """
    Example usage of validation utilities.

    This demonstrates how to validate success criteria with sample data.
    """
    print("Module 3: Success Criteria Validation Example")
    print("=" * 60)

    # Example metrics (replace with actual measurements)
    dataset = DatasetMetrics()
    dataset.start()
    # Simulate: 1247 images generated
    for _ in range(1247):
        dataset.record_image()

    vslam = VSLAMMetrics()
    vslam.start()
    # Simulate: 35.2 Hz VSLAM
    import time
    for _ in range(100):
        vslam.record_frame(pose=(0.0, 0.0, 0.0), num_features=500)
        time.sleep(1.0 / 35.2)

    navigation = NavigationMetrics()
    # Simulate: 10 goals, 97% success, 3.2s planning
    for i in range(10):
        navigation.start_goal()
        navigation.record_planning_time(3.2)
        navigation.end_goal(success=(i < 9), distance_traveled=10.0, num_replans=1)

    # Validate all criteria
    results = validate_all_criteria(
        dataset_metrics=dataset,
        vslam_metrics=vslam,
        navigation_metrics=navigation,
        trajectory_error_percent=1.8,  # SC-003
        completion_rate_percent=92.0,  # SC-005
        quiz_score_percent=85.0,        # SC-006
        perception_accuracy_percent=88.0,  # SC-007
        recovery_success_percent=74.0  # SC-009
    )

    # Print summary
    print(results.get_summary())

    return 0 if results.all_passed else 1


if __name__ == "__main__":
    import sys
    sys.exit(main())
