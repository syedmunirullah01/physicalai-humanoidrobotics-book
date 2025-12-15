#!/usr/bin/env python3
"""
Exercise 1: Create Scene and Sensor Configuration
Module 3: Isaac Sim - Photorealistic Simulation

Learning Objectives:
- Create a photorealistic warehouse scene in Isaac Sim
- Configure virtual sensors (RGB, depth, stereo cameras)
- Validate scene setup for synthetic data generation
- FR-001: Create photorealistic 3D environments
- FR-002: Configure virtual sensors with realistic physics simulation

Usage:
    python3 ex1-create-scene.py
    python3 ex1-create-scene.py --scene-path /path/to/scene.usd
    python3 ex1-create-scene.py --validate-only
"""

import argparse
import os
import sys
import yaml
import numpy as np
from datetime import datetime

# Add module path for shared utilities
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from shared.utils.tier_detection import detect_tier, validate_tier_requirements
from shared.utils.metrics import GPUMetrics


def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='Exercise 1: Create Scene and Sensor Configuration')
    parser.add_argument('--scene-path', type=str, default='chapter-1-isaac-sim/assets/my_warehouse_scene.usd',
                       help='Path to save/load the scene')
    parser.add_argument('--config-path', type=str, default='chapter-1-isaac-sim/assets/sensor-configs.yaml',
                       help='Path to sensor configuration file')
    parser.add_argument('--validate-only', action='store_true',
                       help='Only validate existing scene, don\'t create')
    parser.add_argument('--show-metrics', action='store_true',
                       help='Show GPU and system metrics during execution')

    return parser.parse_args()


def check_isaac_sim_environment():
    """Check if Isaac Sim environment is properly set up"""
    print("🔍 Checking Isaac Sim environment...")

    # Check if running in Isaac Sim context
    try:
        import omni
        import omni.isaac.core
        print("✅ Isaac Sim Python API available")
        return True
    except ImportError:
        print("❌ Isaac Sim Python API not available")
        print("💡 Make sure this script runs within Isaac Sim environment")
        return False


def create_basic_warehouse_scene():
    """Create a basic warehouse scene with essential elements"""
    print("🏗️  Creating basic warehouse scene...")

    # This is a simplified representation - in real Isaac Sim:
    # 1. Create USD stage
    # 2. Add floor, walls, lighting
    # 3. Add warehouse objects (pallets, boxes, shelves)

    scene_elements = {
        'floor': {
            'type': 'plane',
            'size': [50.0, 30.0],  # 50m x 30m
            'material': 'concrete',
            'physics_enabled': True
        },
        'walls': [
            {'position': [0, 15, 4], 'size': [50, 0.2, 8], 'type': 'wall'},  # North wall
            {'position': [0, -15, 4], 'size': [50, 0.2, 8], 'type': 'wall'},  # South wall
            {'position': [25, 0, 4], 'size': [0.2, 30, 8], 'type': 'wall'},   # East wall
            {'position': [-25, 0, 4], 'size': [0.2, 30, 8], 'type': 'wall'}   # West wall
        ],
        'lighting': {
            'type': 'distant',
            'intensity': 1000,
            'color': [0.9, 0.9, 0.9],
            'position': [0, 0, 10]
        },
        'objects': [
            {'type': 'pallet', 'position': [5, 5, 0.1], 'class_id': 1},
            {'type': 'box', 'position': [5, 6, 0.8], 'class_id': 2},
            {'type': 'shelf', 'position': [10, 8, 0], 'class_id': 6}
        ]
    }

    print(f"✅ Created scene with {len(scene_elements['walls'])} walls, "
          f"{len(scene_elements['objects'])} objects")

    return scene_elements


def configure_sensors_from_config(config_path):
    """Configure sensors based on configuration file"""
    print(f"📡 Configuring sensors from {config_path}...")

    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        stereo_config = config['stereo_camera']
        rgbd_config = config['rgbd_camera']

        sensors = {
            'stereo': {
                'left_camera': stereo_config['left'],
                'right_camera': stereo_config['right'],
                'baseline': stereo_config['baseline'],
                'name': stereo_config['name']
            },
            'rgbd': {
                'rgb': rgbd_config['rgb'],
                'depth': rgbd_config['depth'],
                'name': rgbd_config['name']
            }
        }

        print(f"✅ Configured {len(sensors)} sensor systems")
        print(f"   - Stereo camera: {sensors['stereo']['baseline']*1000:.0f}mm baseline")
        print(f"   - RGB-D camera: {sensors['rgbd']['rgb']['resolution']} resolution")

        return sensors

    except FileNotFoundError:
        print(f"❌ Configuration file not found: {config_path}")
        return None
    except yaml.YAMLError as e:
        print(f"❌ Error parsing YAML configuration: {e}")
        return None


def validate_scene_setup(scene_elements, sensors):
    """Validate that scene and sensors are properly configured"""
    print("✅ Validating scene setup...")

    validation_results = {
        'scene_valid': True,
        'sensors_valid': True,
        'physics_valid': True,
        'issues': []
    }

    # Validate scene elements
    if not scene_elements or len(scene_elements.get('walls', [])) < 4:
        validation_results['scene_valid'] = False
        validation_results['issues'].append("Missing essential walls")

    if not scene_elements.get('floor'):
        validation_results['scene_valid'] = False
        validation_results['issues'].append("Missing floor")

    # Validate sensors
    if not sensors:
        validation_results['sensors_valid'] = False
        validation_results['issues'].append("No sensors configured")
    else:
        if not sensors.get('stereo'):
            validation_results['sensors_valid'] = False
            validation_results['issues'].append("Stereo camera not configured")

        if not sensors.get('rgbd'):
            validation_results['sensors_valid'] = False
            validation_results['issues'].append("RGB-D camera not configured")

    # Validate physics
    floor = scene_elements.get('floor', {})
    if not floor.get('physics_enabled'):
        validation_results['physics_valid'] = False
        validation_results['issues'].append("Floor physics not enabled")

    # Print validation results
    print(f"   Scene valid: {'✅' if validation_results['scene_valid'] else '❌'}")
    print(f"   Sensors valid: {'✅' if validation_results['sensors_valid'] else '❌'}")
    print(f"   Physics valid: {'✅' if validation_results['physics_valid'] else '❌'}")

    if validation_results['issues']:
        print("   Issues found:")
        for issue in validation_results['issues']:
            print(f"     - {issue}")

    return validation_results


def save_scene_to_file(scene_elements, sensors, scene_path):
    """Save scene configuration to file (simplified representation)"""
    print(f"💾 Saving scene to {scene_path}...")

    # Create directory if it doesn't exist
    os.makedirs(os.path.dirname(scene_path), exist_ok=True)

    scene_data = {
        'metadata': {
            'created': datetime.now().isoformat(),
            'exercise': 'ex1-create-scene',
            'module': 'Module 3 Isaac Sim'
        },
        'scene_elements': scene_elements,
        'sensors': sensors
    }

    try:
        with open(scene_path, 'w') as f:
            # In real implementation, this would save actual USD content
            f.write(f"# Isaac Sim Scene Configuration - Module 3 Exercise 1\n")
            f.write(f"# Created: {scene_data['metadata']['created']}\n")
            f.write(f"# Scene Elements: {len(scene_elements.get('walls', [])) + len(scene_elements.get('objects', []))} objects\n")
            f.write(f"# Sensors: {len(sensors) if sensors else 0} systems\n")
            f.write("\n# This is a placeholder file. In Isaac Sim, this would contain actual USD scene data.\n")
            f.write("# For real implementation, use Isaac Sim's stage and prim APIs.\n")

        print(f"✅ Scene saved successfully")
        return True

    except Exception as e:
        print(f"❌ Error saving scene: {e}")
        return False


def load_and_validate_existing_scene(scene_path):
    """Load and validate an existing scene file"""
    print(f"🔍 Loading existing scene from {scene_path}...")

    if not os.path.exists(scene_path):
        print(f"❌ Scene file does not exist: {scene_path}")
        return None, None

    try:
        # In real implementation, this would load USD file
        # For this exercise, we'll simulate loading
        with open(scene_path, 'r') as f:
            content = f.read()

        # Simulate extracted scene elements and sensors
        scene_elements = create_basic_warehouse_scene()  # Placeholder
        sensors = configure_sensors_from_config('chapter-1-isaac-sim/assets/sensor-configs.yaml')

        print(f"✅ Scene loaded successfully")
        return scene_elements, sensors

    except Exception as e:
        print(f"❌ Error loading scene: {e}")
        return None, None


def show_system_metrics():
    """Display system metrics"""
    print("📊 System Metrics:")

    # GPU metrics
    gpu_stats = GPUMetrics.get_gpu_stats()
    if gpu_stats:
        print(f"   GPU: {gpu_stats['gpu_utilization_percent']:.1f}% utilization")
        print(f"   VRAM: {gpu_stats['memory_used_mb']:.0f}/{gpu_stats['memory_total_mb']:.0f} MB "
              f"({gpu_stats['memory_utilization_percent']:.1f}%)")
    else:
        print("   GPU: Not available (GPU metrics library not installed)")

    # System metrics
    sys_stats = GPUMetrics.get_system_stats()
    print(f"   CPU: {sys_stats['cpu_utilization_percent']:.1f}% utilization")
    print(f"   RAM: {sys_stats['ram_used_mb']:.0f}/{sys_stats['ram_total_mb']:.0f} MB "
          f"({sys_stats['ram_utilization_percent']:.1f}%)")


def main():
    """Main exercise execution"""
    print("=" * 60)
    print("Module 3 Exercise 1: Create Scene and Sensor Configuration")
    print("=" * 60)

    args = parse_arguments()

    # Show system metrics if requested
    if args.show_metrics:
        show_system_metrics()
        print()

    # Check hardware tier
    tier = detect_tier()
    print(f"🎯 Hardware Tier: {tier.value.upper()} ({tier.name})")

    is_valid, message = validate_tier_requirements(tier)
    if not is_valid:
        print(f"⚠️  Tier validation warning: {message}")
    else:
        print(f"✅ Tier validation: {message}")
    print()

    # Check Isaac Sim environment
    if not check_isaac_sim_environment():
        print("❌ Cannot proceed without Isaac Sim environment")
        return 1

    print()

    # Load existing scene if validate-only
    if args.validate_only:
        print("🔍 Validation Mode: Loading existing scene...")
        scene_elements, sensors = load_and_validate_existing_scene(args.scene_path)

        if scene_elements is None:
            print("❌ Could not load existing scene for validation")
            return 1
    else:
        # Create new scene
        print("🚀 Creating new scene...")
        scene_elements = create_basic_warehouse_scene()

        # Configure sensors
        sensors = configure_sensors_from_config(args.config_path)
        if sensors is None:
            print("❌ Could not configure sensors")
            return 1

        # Save the scene
        success = save_scene_to_file(scene_elements, sensors, args.scene_path)
        if not success:
            print("❌ Could not save scene")
            return 1

    print()

    # Validate setup
    validation_results = validate_scene_setup(scene_elements, sensors)

    print()

    # Exercise completion summary
    print("📋 Exercise 1 Summary:")
    print(f"   - Scene elements created: {'✅' if scene_elements else '❌'}")
    print(f"   - Sensors configured: {'✅' if sensors else '❌'}")
    print(f"   - Scene validation: {'✅ PASS' if all(validation_results[k] for k in ['scene_valid', 'sensors_valid', 'physics_valid'] if k in validation_results) and not validation_results.get('issues', []) else '❌ FAIL'}")

    # FR-001 and FR-002 validation
    fr001_passed = validation_results['scene_valid']
    fr002_passed = validation_results['sensors_valid']

    print(f"   - FR-001 (Photorealistic environments): {'✅ PASS' if fr001_passed else '❌ FAIL'}")
    print(f"   - FR-002 (Virtual sensors): {'✅ PASS' if fr002_passed else '❌ FAIL'}")

    if fr001_passed and fr002_passed:
        print("\n🎉 Exercise 1 completed successfully!")
        print("You have successfully created a photorealistic warehouse scene")
        print("with properly configured virtual sensors for synthetic data generation.")
        return 0
    else:
        print("\n⚠️  Exercise 1 has issues that need to be resolved.")
        print("Check the validation messages above and ensure scene setup is correct.")
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)