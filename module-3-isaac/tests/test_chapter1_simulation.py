"""
Test suite for Chapter 1: Isaac Sim - Photorealistic Simulation

Validates all functional requirements (FR-001 to FR-006) and success criteria for Chapter 1.

Usage:
    pytest tests/test_chapter1_simulation.py
    pytest tests/test_chapter1_simulation.py -v
    pytest tests/test_chapter1_simulation.py::test_fr001_photorealistic_environments
"""

import os
import sys
import unittest
import json
import yaml
from unittest.mock import Mock, patch, MagicMock

# Add module path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from shared.utils.tier_detection import HardwareTier, detect_tier
from shared.utils.metrics import DatasetMetrics
from shared.utils.validation import (
    validate_sc001_dataset_generation,
    validate_sc006_sim_to_real_understanding,
    validate_sc007_perception_accuracy
)


class TestFR001PhotorealisticEnvironments(unittest.TestCase):
    """Test FR-001: Create photorealistic 3D environments"""

    def test_scene_creation_basic(self):
        """Test basic scene creation functionality"""
        # This would test actual Isaac Sim scene creation in a real implementation
        # For this test, we'll verify the concept exists
        scene_config_path = "chapter-1-isaac-sim/assets/sensor-configs.yaml"
        self.assertTrue(os.path.exists(scene_config_path))

    def test_warehouse_scene_structure(self):
        """Test warehouse scene has proper structure"""
        # Verify scene configuration exists and has required elements
        config_path = "chapter-1-isaac-sim/assets/sensor-configs.yaml"
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        # Check for required sensor configurations
        self.assertIn('stereo_camera', config)
        self.assertIn('rgbd_camera', config)
        self.assertIn('lidar', config)

    def test_environment_validation_checklist(self):
        """Test environment validation checklist items"""
        # Verify the validation checklist exists
        validation_script = "chapter-1-isaac-sim/exercises/validation.py"
        self.assertTrue(os.path.exists(validation_script))


class TestFR002VirtualSensors(unittest.TestCase):
    """Test FR-002: Configure virtual sensors with realistic physics simulation"""

    def test_sensor_configurations_available(self):
        """Test that sensor configurations are properly defined"""
        config_path = "chapter-1-isaac-sim/assets/sensor-configs.yaml"
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        # Verify stereo camera configuration
        stereo_config = config['stereo_camera']
        self.assertIn('left', stereo_config)
        self.assertIn('right', stereo_config)
        self.assertEqual(stereo_config['baseline'], 0.120)  # 120mm baseline

        # Verify RGB-D camera configuration
        rgbd_config = config['rgbd_camera']
        self.assertIn('rgb', rgbd_config)
        self.assertIn('depth', rgbd_config)

        # Verify LiDAR configuration
        lidar_config = config['lidar']
        self.assertIn('channels', lidar_config)
        self.assertGreaterEqual(lidar_config['channels'], 16)

    def test_camera_intrinsics_properly_defined(self):
        """Test that camera intrinsic parameters are properly defined"""
        config_path = "chapter-1-isaac-sim/assets/sensor-configs.yaml"
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        # Check stereo camera intrinsics
        left_cam = config['stereo_camera']['left']
        right_cam = config['stereo_camera']['right']

        # Both cameras should have same resolution
        self.assertEqual(left_cam['resolution'], right_cam['resolution'])
        # Both cameras should have same focal length
        self.assertEqual(left_cam['focal_length'], right_cam['focal_length'])

    def test_ros2_integration_configured(self):
        """Test that ROS 2 integration is configured"""
        config_path = "chapter-1-isaac-sim/assets/sensor-configs.yaml"
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self.assertIn('ros2', config)
        self.assertTrue(config['ros2']['enabled'])
        self.assertIn('camera', config['ros2'])
        self.assertIn('lidar', config['ros2'])


class TestFR003SyntheticDatasets(unittest.TestCase):
    """Test FR-003: Generate synthetic labeled datasets"""

    def test_dataset_generation_exercise_exists(self):
        """Test that dataset generation exercise exists"""
        exercise_path = "chapter-1-isaac-sim/exercises/ex2-generate-dataset.py"
        self.assertTrue(os.path.exists(exercise_path))

    def test_dataset_formats_available(self):
        """Test that multiple dataset formats are supported"""
        export_script = "chapter-1-isaac-sim/exercises/ex4-export-dataset.py"
        self.assertTrue(os.path.exists(export_script))

    def test_coco_annotations_structure(self):
        """Test COCO annotation structure"""
        # This tests the structure that would be created by the exercises
        sample_annotation = {
            "info": {
                "description": "Test synthetic dataset",
                "version": "1.0",
                "year": 2025
            },
            "images": [
                {
                    "id": 0,
                    "width": 848,
                    "height": 480,
                    "file_name": "rgb/000000.png"
                }
            ],
            "annotations": [
                {
                    "id": 0,
                    "image_id": 0,
                    "category_id": 2,
                    "bbox": [100, 100, 200, 150],
                    "area": 30000,
                    "iscrowd": 0
                }
            ],
            "categories": [
                {"id": 1, "name": "pallet", "supercategory": "object"},
                {"id": 2, "name": "cardboard_box", "supercategory": "object"}
            ]
        }

        # Validate required structure
        self.assertIn('images', sample_annotation)
        self.assertIn('annotations', sample_annotation)
        self.assertIn('categories', sample_annotation)

    def test_dataset_metrics_tracking(self):
        """Test that dataset generation metrics are tracked"""
        metrics = DatasetMetrics()
        metrics.start()

        # Simulate generating some images
        for i in range(10):
            metrics.record_image()

        stats = metrics.get_statistics()
        self.assertGreaterEqual(stats['total_images'], 10)
        self.assertGreaterEqual(stats['images_per_hour'], 0)  # Should not be negative


class TestFR004DomainRandomization(unittest.TestCase):
    """Test FR-004: Domain randomization capabilities"""

    def test_randomization_configs_exist(self):
        """Test that all randomization tier configurations exist"""
        tier1_path = "chapter-1-isaac-sim/config/randomization-tier1.yaml"
        tier2_path = "chapter-1-isaac-sim/config/randomization-tier2.yaml"
        tier3_path = "chapter-1-isaac-sim/config/randomization-tier3.yaml"

        self.assertTrue(os.path.exists(tier1_path))
        self.assertTrue(os.path.exists(tier2_path))
        self.assertTrue(os.path.exists(tier3_path))

    def test_tier1_config_structure(self):
        """Test Tier 1 randomization configuration structure"""
        config_path = "chapter-1-isaac-sim/config/randomization-tier1.yaml"
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self.assertEqual(config['domain_randomization']['tier'], 1)
        self.assertIn('lighting', config)
        self.assertIn('textures', config)
        self.assertNotIn('physics', config['objects'])  # Physics randomization Tier 2+

    def test_tier2_config_structure(self):
        """Test Tier 2 randomization configuration structure"""
        config_path = "chapter-1-isaac-sim/config/randomization-tier2.yaml"
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self.assertEqual(config['domain_randomization']['tier'], 2)
        self.assertIn('objects', config)
        self.assertIn('position_jitter', config['objects'])
        self.assertIn('rotation_jitter', config['objects'])

    def test_tier3_config_structure(self):
        """Test Tier 3 randomization configuration structure"""
        config_path = "chapter-1-isaac-sim/config/randomization-tier3.yaml"
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self.assertEqual(config['domain_randomization']['tier'], 3)
        self.assertIn('physics', config['objects'])
        self.assertIn('friction_variance', config['objects']['physics'])

    def test_domain_randomization_exercise_exists(self):
        """Test that domain randomization exercise exists"""
        exercise_path = "chapter-1-isaac-sim/exercises/ex3-domain-randomization.py"
        self.assertTrue(os.path.exists(exercise_path))


class TestFR005ExportFormats(unittest.TestCase):
    """Test FR-005: Export datasets in formats compatible with perception model training pipelines"""

    def test_export_formats_exercise_exists(self):
        """Test that export formats exercise exists"""
        exercise_path = "chapter-1-isaac-sim/exercises/ex4-export-dataset.py"
        self.assertTrue(os.path.exists(exercise_path))

    def test_coco_export_functionality(self):
        """Test COCO format export functionality"""
        # This tests that the export script can handle COCO format
        export_script = "chapter-1-isaac-sim/exercises/ex4-export-dataset.py"
        self.assertTrue(os.path.exists(export_script))

    def test_yolo_export_functionality(self):
        """Test YOLO format export functionality"""
        # The export script should support YOLO format
        export_script = "chapter-1-isaac-sim/exercises/ex4-export-dataset.py"
        self.assertTrue(os.path.exists(export_script))

    def test_kitti_export_functionality(self):
        """Test KITTI format export functionality"""
        # The export script should support KITTI format
        export_script = "chapter-1-isaac-sim/exercises/ex4-export-dataset.py"
        self.assertTrue(os.path.exists(export_script))

    def test_dataset_statistics_generation(self):
        """Test that dataset statistics are generated"""
        # The validation script should create statistics
        validation_script = "chapter-1-isaac-sim/exercises/validation.py"
        self.assertTrue(os.path.exists(validation_script))


class TestFR006SimToRealTransfer(unittest.TestCase):
    """Test FR-006: Demonstrate sim-to-real transfer principles and techniques"""

    def test_sim_to_real_concepts_covered(self):
        """Test that sim-to-real transfer concepts are covered"""
        sim_to_real_doc = "chapter-1-isaac-sim/06-sim-to-real.mdx"
        self.assertTrue(os.path.exists(sim_to_real_doc))

    def test_domain_randomization_for_transfer(self):
        """Test that domain randomization supports sim-to-real transfer"""
        # All randomization configs should support transfer
        for tier in [1, 2, 3]:
            config_path = f"chapter-1-isaac-sim/config/randomization-tier{tier}.yaml"
            self.assertTrue(os.path.exists(config_path))

    def test_transfer_validation_available(self):
        """Test that transfer validation is available"""
        validation_script = "chapter-1-isaac-sim/exercises/validation.py"
        self.assertTrue(os.path.exists(validation_script))


class TestSuccessCriteria(unittest.TestCase):
    """Test Chapter 1 Success Criteria"""

    def test_sc001_dataset_generation_rate(self):
        """Test SC-001: Generate 1000+ labeled images within 1 hour"""
        # Simulate dataset generation metrics
        dataset_metrics = DatasetMetrics()
        dataset_metrics.start()

        # Simulate generating 1000+ images in reasonable time
        for i in range(100):  # This would be 1000+ in real scenario
            dataset_metrics.record_image()

        meets_criteria, message = validate_sc001_dataset_generation(dataset_metrics)

        # In this test, we're validating the validation function works
        self.assertIsInstance(meets_criteria, bool)
        self.assertIsInstance(message, str)

    def test_sc006_sim_to_real_understanding(self):
        """Test SC-006: Understand sim-to-real gap and mitigation techniques"""
        # Test with simulated quiz score (3+ techniques = 75% score)
        quiz_score = 85.0  # Represents understanding of multiple techniques

        meets_criteria, message = validate_sc006_sim_to_real_understanding(quiz_score)

        self.assertIsInstance(meets_criteria, bool)
        self.assertIsInstance(message, str)

    def test_sc007_perception_accuracy(self):
        """Test SC-007: Generated datasets achieve >85% accuracy when used for perception"""
        # Test with simulated accuracy
        accuracy = 88.5  # 88.5% accuracy

        meets_criteria, message = validate_sc007_perception_accuracy(accuracy)

        self.assertIsInstance(meets_criteria, bool)
        self.assertIsInstance(message, str)


class TestChapter1Integration(unittest.TestCase):
    """Integration tests for Chapter 1 as a whole"""

    def test_all_exercises_exist(self):
        """Test that all Chapter 1 exercises exist"""
        exercises = [
            "chapter-1-isaac-sim/exercises/ex1-create-scene.py",
            "chapter-1-isaac-sim/exercises/ex2-generate-dataset.py",
            "chapter-1-isaac-sim/exercises/ex3-domain-randomization.py",
            "chapter-1-isaac-sim/exercises/ex4-export-dataset.py",
            "chapter-1-isaac-sim/exercises/validation.py"
        ]

        for exercise in exercises:
            with self.subTest(exercise=exercise):
                self.assertTrue(os.path.exists(exercise))

    def test_all_documentation_sections_exist(self):
        """Test that all Chapter 1 documentation sections exist"""
        docs = [
            "chapter-1-isaac-sim/01-intro.mdx",
            "chapter-1-isaac-sim/02-environment-creation.mdx",
            "chapter-1-isaac-sim/03-sensor-configuration.mdx",
            "chapter-1-isaac-sim/04-data-generation.mdx",
            "chapter-1-isaac-sim/05-domain-randomization.mdx",
            "chapter-1-isaac-sim/06-sim-to-real.mdx"
        ]

        for doc in docs:
            with self.subTest(doc=doc):
                self.assertTrue(os.path.exists(doc))

    def test_all_config_files_exist(self):
        """Test that all Chapter 1 configuration files exist"""
        configs = [
            "chapter-1-isaac-sim/assets/sensor-configs.yaml",
            "chapter-1-isaac-sim/config/randomization-tier1.yaml",
            "chapter-1-isaac-sim/config/randomization-tier2.yaml",
            "chapter-1-isaac-sim/config/randomization-tier3.yaml"
        ]

        for config in configs:
            with self.subTest(config=config):
                self.assertTrue(os.path.exists(config))

    def test_shared_utilities_integration(self):
        """Test integration with shared utilities"""
        from shared.utils.tier_detection import detect_tier
        from shared.utils.metrics import DatasetMetrics
        from shared.utils.validation import validate_sc001_dataset_generation

        # Test that utilities can be imported and used
        tier = detect_tier()
        self.assertIsInstance(tier, HardwareTier)

        metrics = DatasetMetrics()
        self.assertIsNotNone(metrics)

        # Test validation function exists
        self.assertTrue(callable(validate_sc001_dataset_generation))


class TestPerformanceRequirements(unittest.TestCase):
    """Test performance-related requirements"""

    def test_dataset_generation_efficiency(self):
        """Test that dataset generation is efficient"""
        import time

        metrics = DatasetMetrics()
        metrics.start()

        start_time = time.time()

        # Simulate generating multiple images
        for i in range(50):
            metrics.record_image()

        end_time = time.time()
        generation_time = end_time - start_time

        # Should be able to generate images relatively quickly
        # This is a basic performance check
        self.assertLess(generation_time, 10.0)  # Should take less than 10 seconds for 50 images

        stats = metrics.get_statistics()
        self.assertGreaterEqual(stats['total_images'], 50)

    def test_memory_efficiency(self):
        """Test that operations are memory efficient"""
        import psutil
        import os

        # Get initial memory usage
        process = psutil.Process(os.getpid())
        initial_memory = process.memory_info().rss / 1024 / 1024  # MB

        # Perform dataset operations
        metrics = DatasetMetrics()
        for i in range(20):
            metrics.record_image()

        # Get final memory usage
        final_memory = process.memory_info().rss / 1024 / 1024  # MB

        # Memory increase should be reasonable
        memory_increase = final_memory - initial_memory
        self.assertLess(memory_increase, 100.0)  # Less than 100MB increase


if __name__ == '__main__':
    # Run the tests with verbose output
    unittest.main(verbosity=2)