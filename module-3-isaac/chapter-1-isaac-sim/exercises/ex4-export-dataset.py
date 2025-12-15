#!/usr/bin/env python3
"""
Exercise 4: Export Dataset in Standard Formats
Module 3: Isaac Sim - Photorealistic Simulation

Learning Objectives:
- Export synthetic datasets in COCO, YOLO, and KITTI formats
- Validate dataset compatibility with perception frameworks
- Create dataset metadata and statistics
- FR-005: Export datasets in formats compatible with perception model training pipelines

Usage:
    python3 ex4-export-dataset.py
    python3 ex4-export-dataset.py --input-dir shared/datasets/generated_exercise2 --output-dir exported_datasets
    python3 ex4-export-dataset.py --formats coco,yolo,kitti --validate-only
"""

import argparse
import os
import sys
import json
import yaml
import numpy as np
import cv2
import shutil
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Tuple

# Add module path for shared utilities
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from shared.utils.tier_detection import detect_tier
from shared.utils.metrics import GPUMetrics
from shared.utils.validation import validate_sc007_perception_accuracy


def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='Exercise 4: Export Dataset in Standard Formats')
    parser.add_argument('--input-dir', type=str, default='shared/datasets/generated_exercise2',
                       help='Input directory containing raw dataset')
    parser.add_argument('--output-dir', type=str, default='shared/datasets/exported_exercise4',
                       help='Output directory for exported datasets')
    parser.add_argument('--formats', type=str, default='coco,yolo',
                       help='Comma-separated list of formats to export (coco, yolo, kitti)')
    parser.add_argument('--validate-only', action='store_true',
                       help='Only validate existing exported dataset')
    parser.add_argument('--show-metrics', action='store_true',
                       help='Show performance metrics during export')
    parser.add_argument('--create-symlinks', action='store_true',
                       help='Create symlinks instead of copying files (faster)')

    return parser.parse_args()


def validate_input_dataset(input_dir):
    """Validate that input dataset has required structure"""
    print(f"🔍 Validating input dataset in {input_dir}...")

    required_dirs = ['rgb', 'depth', 'segmentation']
    required_files = ['labels.json']  # COCO format annotations

    issues = []

    # Check required directories
    for dir_name in required_dirs:
        dir_path = os.path.join(input_dir, dir_name)
        if not os.path.exists(dir_path):
            issues.append(f"Missing directory: {dir_path}")
        else:
            files = [f for f in os.listdir(dir_path) if f.lower().endswith(('.png', '.jpg', '.jpeg'))]
            if len(files) == 0:
                issues.append(f"No image files in {dir_path}")

    # Check required files
    for file_name in required_files:
        file_path = os.path.join(input_dir, file_name)
        if not os.path.exists(file_path):
            issues.append(f"Missing file: {file_path}")
        else:
            # Validate COCO format
            try:
                with open(file_path, 'r') as f:
                    data = json.load(f)

                required_keys = ['images', 'annotations', 'categories']
                for key in required_keys:
                    if key not in data:
                        issues.append(f"Missing COCO key: {key}")
            except json.JSONDecodeError:
                issues.append(f"Invalid JSON in {file_path}")

    print(f"   Input validation: {'✅ PASS' if not issues else '❌ FAIL'}")
    if issues:
        for issue in issues:
            print(f"     - {issue}")

    return len(issues) == 0, issues


def load_coco_annotations(input_dir):
    """Load COCO format annotations"""
    annotations_path = os.path.join(input_dir, 'labels.json')

    try:
        with open(annotations_path, 'r') as f:
            coco_data = json.load(f)
        return coco_data
    except Exception as e:
        print(f"❌ Error loading COCO annotations: {e}")
        return None


def convert_coco_to_yolo(coco_data, image_width=848, image_height=480):
    """Convert COCO annotations to YOLO format"""
    print("🔄 Converting COCO to YOLO format...")

    # Create mapping from COCO category IDs to consecutive YOLO IDs
    category_map = {}
    for i, cat in enumerate(coco_data['categories']):
        category_map[cat['id']] = i

    # Convert each image's annotations
    yolo_annotations = {}

    for img in coco_data['images']:
        img_id = img['id']
        img_annotations = [ann for ann in coco_data['annotations'] if ann['image_id'] == img_id]

        yolo_lines = []
        for ann in img_annotations:
            # Convert COCO bbox [x, y, width, height] to YOLO format
            coco_bbox = ann['bbox']

            # Normalize coordinates to [0, 1]
            center_x = (coco_bbox[0] + coco_bbox[2] / 2) / image_width
            center_y = (coco_bbox[1] + coco_bbox[3] / 2) / image_height
            width = coco_bbox[2] / image_width
            height = coco_bbox[3] / image_height

            # Get YOLO class ID
            yolo_class_id = category_map[ann['category_id']]

            # Create YOLO line: class_id center_x center_y width height
            yolo_line = f"{yolo_class_id} {center_x:.6f} {center_y:.6f} {width:.6f} {height:.6f}"
            yolo_lines.append(yolo_line)

        yolo_annotations[f"{img_id:06d}.txt"] = yolo_lines

    print(f"✅ Converted annotations for {len(coco_data['images'])} images")
    return yolo_annotations, category_map


def convert_coco_to_kitti(coco_data):
    """Convert COCO annotations to KITTI format"""
    print("🔄 Converting COCO to KITTI format...")

    kitti_annotations = {}

    # KITTI class mapping (simplified)
    coco_to_kitti = {
        1: 'Car',      # pallet -> car
        2: 'Van',      # cardboard_box -> van
        3: 'Truck',    # barrel -> truck
        4: 'Van',      # forklift -> van
        5: 'Pedestrian',  # person -> pedestrian
        6: 'DontCare'    # shelf -> don't care
    }

    for img in coco_data['images']:
        img_id = img['id']
        img_annotations = [ann for ann in coco_data['annotations'] if ann['image_id'] == img_id]

        kitti_lines = []
        for ann in img_annotations:
            # Get KITTI class name
            coco_class_id = ann['category_id']
            kitti_class = coco_to_kitti.get(coco_class_id, 'DontCare')

            # KITTI format: type, truncated, occluded, alpha, bbox, dimensions, location, rotation_y, score
            # For synthetic data, we'll use default values
            coco_bbox = ann['bbox']
            kitti_bbox = [coco_bbox[0], coco_bbox[1], coco_bbox[0] + coco_bbox[2], coco_bbox[1] + coco_bbox[3]]

            # Default values for KITTI format
            truncated = 0.0
            occluded = 0
            alpha = -10
            dimensions = "1.0 1.0 1.0"  # height, width, length (dummy values)
            location = "0.0 0.0 0.0"    # x, y, z (dummy values)
            rotation_y = 0.0
            score = 1.0

            kitti_line = (f"{kitti_class} {truncated} {occluded} {alpha} "
                         f"{kitti_bbox[0]:.2f} {kitti_bbox[1]:.2f} {kitti_bbox[2]:.2f} {kitti_bbox[3]:.2f} "
                         f"{dimensions} {location} {rotation_y:.2f} {score:.2f}")

            kitti_lines.append(kitti_line)

        kitti_annotations[f"{img_id:06d}.txt"] = kitti_lines

    print(f"✅ Converted annotations for {len(coco_data['images'])} images")
    return kitti_annotations


def export_to_coco_format(coco_data, output_dir):
    """Export dataset in COCO format"""
    print("📦 Exporting COCO format...")

    coco_output_dir = os.path.join(output_dir, 'coco')
    os.makedirs(coco_output_dir, exist_ok=True)

    # Copy images
    input_rgb_dir = os.path.join(os.path.dirname(output_dir.replace('exported', 'generated_exercise2')), 'rgb')
    output_img_dir = os.path.join(coco_output_dir, 'images')
    os.makedirs(output_img_dir, exist_ok=True)

    if os.path.exists(input_rgb_dir):
        for img_file in os.listdir(input_rgb_dir):
            if img_file.lower().endswith(('.png', '.jpg', '.jpeg')):
                src = os.path.join(input_rgb_dir, img_file)
                dst = os.path.join(output_img_dir, img_file)
                if args.create_symlinks and hasattr(os, 'symlink'):
                    os.symlink(src, dst)
                else:
                    shutil.copy2(src, dst)

    # Save annotations
    annotations_path = os.path.join(coco_output_dir, 'annotations.json')
    with open(annotations_path, 'w') as f:
        json.dump(coco_data, f, indent=2)

    print(f"✅ COCO export completed: {len(coco_data['images'])} images")


def export_to_yolo_format(yolo_annotations, category_map, input_dir, output_dir):
    """Export dataset in YOLO format"""
    print("📦 Exporting YOLO format...")

    yolo_output_dir = os.path.join(output_dir, 'yolo')
    os.makedirs(yolo_output_dir, exist_ok=True)

    # Create images and labels subdirectories
    img_output_dir = os.path.join(yolo_output_dir, 'images')
    labels_output_dir = os.path.join(yolo_output_dir, 'labels')
    os.makedirs(img_output_dir, exist_ok=True)
    os.makedirs(labels_output_dir, exist_ok=True)

    # Copy images
    input_rgb_dir = os.path.join(input_dir, 'rgb')
    if os.path.exists(input_rgb_dir):
        for img_file in os.listdir(input_rgb_dir):
            if img_file.lower().endswith(('.png', '.jpg', '.jpeg')):
                src = os.path.join(input_rgb_dir, img_file)
                dst = os.path.join(img_output_dir, img_file)
                if args.create_symlinks and hasattr(os, 'symlink'):
                    os.symlink(src, dst)
                else:
                    shutil.copy2(src, dst)

    # Save YOLO annotation files
    for filename, annotations in yolo_annotations.items():
        label_path = os.path.join(labels_output_dir, filename)
        with open(label_path, 'w') as f:
            f.write('\n'.join(annotations))

    # Create YOLO dataset configuration file
    classes = [cat['name'] for cat in load_coco_annotations(input_dir)['categories']]
    yolo_config = {
        'path': './',  # Root path
        'train': 'images/',  # Training images path
        'val': 'images/',    # Validation images path (same for this example)
        'nc': len(classes),  # Number of classes
        'names': classes     # Class names
    }

    config_path = os.path.join(yolo_output_dir, 'dataset.yaml')
    with open(config_path, 'w') as f:
        yaml.dump(yolo_config, f, default_flow_style=False)

    print(f"✅ YOLO export completed: {len(yolo_annotations)} annotation files")


def export_to_kitti_format(kitti_annotations, input_dir, output_dir):
    """Export dataset in KITTI format"""
    print("📦 Exporting KITTI format...")

    kitti_output_dir = os.path.join(output_dir, 'kitti')
    os.makedirs(kitti_output_dir, exist_ok=True)

    # Create KITTI directory structure
    img_output_dir = os.path.join(kitti_output_dir, 'image_2')  # KITTI standard name
    labels_output_dir = os.path.join(kitti_output_dir, 'label_2')  # KITTI standard name
    os.makedirs(img_output_dir, exist_ok=True)
    os.makedirs(labels_output_dir, exist_ok=True)

    # Copy images
    input_rgb_dir = os.path.join(input_dir, 'rgb')
    if os.path.exists(input_rgb_dir):
        for img_file in os.listdir(input_rgb_dir):
            if img_file.lower().endswith(('.png', '.jpg', '.jpeg')):
                src = os.path.join(input_rgb_dir, img_file)
                dst = os.path.join(img_output_dir, img_file)
                if args.create_symlinks and hasattr(os, 'symlink'):
                    os.symlink(src, dst)
                else:
                    shutil.copy2(src, dst)

    # Save KITTI annotation files
    for filename, annotations in kitti_annotations.items():
        label_path = os.path.join(labels_output_dir, filename.replace('.txt', '.txt'))  # Keep .txt extension
        with open(label_path, 'w') as f:
            f.write('\n'.join(annotations))

    print(f"✅ KITTI export completed: {len(kitti_annotations)} annotation files")


def create_dataset_statistics(coco_data, output_dir):
    """Create dataset statistics and metadata"""
    print("📊 Creating dataset statistics...")

    stats = {
        'dataset_info': {
            'name': 'Module 3 Isaac Sim Synthetic Dataset',
            'version': '1.0',
            'created': datetime.now().isoformat(),
            'generator': 'Isaac Sim Domain Randomization Exercise 4'
        },
        'statistics': {
            'total_images': len(coco_data['images']),
            'total_annotations': len(coco_data['annotations']),
            'categories': len(coco_data['categories']),
            'average_annotations_per_image': len(coco_data['annotations']) / len(coco_data['images']) if coco_data['images'] else 0
        },
        'category_distribution': {},
        'image_statistics': {
            'width': 848,  # From our generated images
            'height': 480,
            'format': 'PNG'
        }
    }

    # Calculate category distribution
    for cat in coco_data['categories']:
        cat_id = cat['id']
        cat_name = cat['name']
        count = sum(1 for ann in coco_data['annotations'] if ann['category_id'] == cat_id)
        stats['category_distribution'][cat_name] = count

    # Save statistics
    stats_path = os.path.join(output_dir, 'dataset_statistics.json')
    with open(stats_path, 'w') as f:
        json.dump(stats, f, indent=2)

    print(f"✅ Statistics created for {stats['statistics']['total_images']} images")


def validate_exported_dataset(output_dir, formats):
    """Validate that exported datasets are properly formatted"""
    print("🔍 Validating exported datasets...")

    issues = []

    for format_name in formats:
        format_dir = os.path.join(output_dir, format_name)
        if not os.path.exists(format_dir):
            issues.append(f"Missing export directory for {format_name}")
            continue

        if format_name == 'coco':
            ann_file = os.path.join(format_dir, 'annotations.json')
            if not os.path.exists(ann_file):
                issues.append(f"Missing COCO annotations in {format_dir}")
            else:
                try:
                    with open(ann_file, 'r') as f:
                        json.load(f)
                except json.JSONDecodeError:
                    issues.append(f"Invalid JSON in COCO annotations: {ann_file}")

        elif format_name == 'yolo':
            # Check for dataset.yaml and labels directory
            config_file = os.path.join(format_dir, 'dataset.yaml')
            labels_dir = os.path.join(format_dir, 'labels')
            if not os.path.exists(config_file):
                issues.append(f"Missing YOLO config in {format_dir}")
            if not os.path.exists(labels_dir):
                issues.append(f"Missing YOLO labels directory in {format_dir}")

        elif format_name == 'kitti':
            labels_dir = os.path.join(format_dir, 'label_2')
            if not os.path.exists(labels_dir):
                issues.append(f"Missing KITTI labels directory in {format_dir}")

    print(f"   Export validation: {'✅ PASS' if not issues else '❌ FAIL'}")
    if issues:
        for issue in issues:
            print(f"     - {issue}")

    return len(issues) == 0, issues


def show_perception_framework_compatibility():
    """Show compatibility with perception frameworks"""
    print("\n🔗 Perception Framework Compatibility:")
    print("   COCO Format:")
    print("     • Compatible with: Detectron2, MMDetection, TensorFlow Object Detection API")
    print("     • Used by: Facebook, FAIR, academic research")
    print("     • Features: Segmentation masks, keypoints, complex annotations")
    print()
    print("   YOLO Format:")
    print("     • Compatible with: YOLOv5, YOLOv8, YOLOv9, RT-DETR")
    print("     • Used by: Ultralytics, industrial applications")
    print("     • Features: Simple, fast training and inference")
    print()
    print("   KITTI Format:")
    print("     • Compatible with: KITTI evaluation tools, PCL, OpenPCV")
    print("     • Used by: Autonomous driving research")
    print("     • Features: 3D bounding boxes, multi-modal data")


def main():
    """Main exercise execution"""
    print("=" * 70)
    print("Module 3 Exercise 4: Export Dataset in Standard Formats")
    print("=" * 70)

    args = parse_arguments()
    export_formats = [f.strip() for f in args.formats.split(',')]

    # Show system metrics if requested
    if args.show_metrics:
        gpu_stats = GPUMetrics.get_gpu_stats()
        if gpu_stats:
            print(f"GPU: {gpu_stats['gpu_utilization_percent']:.1f}% utilization, "
                  f"{gpu_stats['memory_used_mb']:.0f}/{gpu_stats['memory_total_mb']:.0f} MB VRAM")
        print()

    # Check hardware tier
    tier = detect_tier()
    print(f"🎯 Hardware Tier: {tier.value.upper()} ({tier.name})")
    print(f"📋 Export formats: {', '.join(export_formats).upper()}")
    print(f"📁 Input: {args.input_dir}")
    print(f"📁 Output: {args.output_dir}")
    print()

    if args.validate_only:
        print(f"🔍 Validation Mode: Validating exported dataset in {args.output_dir}")

        # Validate existing export
        is_valid, issues = validate_exported_dataset(args.output_dir, export_formats)

        # Show compatibility info
        show_perception_framework_compatibility()

        return 0 if is_valid else 1

    # Validate input dataset
    input_valid, input_issues = validate_input_dataset(args.input_dir)
    if not input_valid:
        print("❌ Input dataset validation failed")
        return 1

    # Load COCO annotations
    coco_data = load_coco_annotations(args.input_dir)
    if coco_data is None:
        print("❌ Could not load input annotations")
        return 1

    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)
    print(f"📁 Output directory created: {args.output_dir}")

    print(f"\n🚀 Exporting dataset in {len(export_formats)} formats: {', '.join(export_formats).upper()}")

    # Process each export format
    for fmt in export_formats:
        if fmt.lower() == 'coco':
            export_to_coco_format(coco_data, args.output_dir)
        elif fmt.lower() == 'yolo':
            yolo_annotations, category_map = convert_coco_to_yolo(coco_data)
            export_to_yolo_format(yolo_annotations, category_map, args.input_dir, args.output_dir)
        elif fmt.lower() == 'kitti':
            kitti_annotations = convert_coco_to_kitti(coco_data)
            export_to_kitti_format(kitti_annotations, args.input_dir, args.output_dir)
        else:
            print(f"⚠️  Unknown format: {fmt}. Skipping.")

    # Create dataset statistics
    create_dataset_statistics(coco_data, args.output_dir)

    # Validate exported dataset
    export_valid, export_issues = validate_exported_dataset(args.output_dir, export_formats)

    # Show compatibility info
    show_perception_framework_compatibility()

    # FR-005 validation: Export datasets in formats compatible with perception pipelines
    fr005_passed = export_valid
    print(f"\n📋 Functional Requirements:")
    print(f"   - FR-005 (Export formats): {'✅ PASS' if fr005_passed else '❌ FAIL'}")

    # SC-007 validation: This would require actual model training
    # For this exercise, we'll note that SC-007 relates to perception accuracy
    print(f"SC-007 Validation: 📊 Perception accuracy validation requires model training")
    print(f"                   Exported formats are compatible with standard perception pipelines")

    print(f"\n📁 Exported datasets saved to: {args.output_dir}")
    for fmt in export_formats:
        fmt_dir = os.path.join(args.output_dir, fmt.lower())
        if os.path.exists(fmt_dir):
            print(f"   {fmt.upper()}: {fmt_dir}")

    if export_valid and fr005_passed:
        print(f"\n🎉 Exercise 4 completed successfully!")
        print(f"You have exported the synthetic dataset in {len(export_formats)} standard formats")
        print(f"compatible with major perception frameworks.")
        return 0
    else:
        print(f"\n⚠️  Exercise 4 has issues that need to be resolved.")
        print(f"Check the validation messages above.")
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)