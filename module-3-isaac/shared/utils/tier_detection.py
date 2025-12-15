"""
Tier Detection Utility for Module 3: The AI-Robot Brain (NVIDIA Isaac™)

Automatically detects hardware tier based on available resources:
- Tier A: Simulation Only (desktop/laptop with NVIDIA GPU)
- Tier B: Edge AI (NVIDIA Jetson Orin Nano/NX)
- Tier C: Physical Robot (with humanoid robot hardware)

Usage:
    python3 tier_detection.py

    # Or import in code:
    from shared.utils.tier_detection import detect_tier, get_gpu_info

    tier = detect_tier()
    print(f"Detected tier: {tier}")
"""

import os
import platform
import subprocess
import sys
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple


class HardwareTier(Enum):
    """Hardware tiers for Module 3 learning paths"""
    TIER_A = "simulation"  # Simulation only
    TIER_B = "jetson"      # Edge AI (Jetson Orin)
    TIER_C = "hardware"    # Physical humanoid robot
    UNKNOWN = "unknown"


@dataclass
class GPUInfo:
    """GPU information for tier detection"""
    name: str
    vram_mb: int
    cuda_version: str
    compute_capability: str
    is_jetson: bool
    is_available: bool


def get_gpu_info() -> Optional[GPUInfo]:
    """
    Detect NVIDIA GPU and retrieve specifications.

    Returns:
        GPUInfo object if NVIDIA GPU detected, None otherwise
    """
    try:
        # Try nvidia-smi first
        result = subprocess.run(
            ['nvidia-smi', '--query-gpu=name,memory.total,driver_version', '--format=csv,noheader,nounits'],
            capture_output=True,
            text=True,
            timeout=5
        )

        if result.returncode == 0:
            gpu_data = result.stdout.strip().split(',')
            if len(gpu_data) >= 2:
                gpu_name = gpu_data[0].strip()
                vram_mb = int(float(gpu_data[1].strip()))
                driver_version = gpu_data[2].strip() if len(gpu_data) > 2 else "Unknown"

                # Check if Jetson
                is_jetson = 'tegra' in platform.platform().lower() or 'jetson' in gpu_name.lower()

                # Get CUDA version
                cuda_version = get_cuda_version()

                # Get compute capability
                compute_capability = get_compute_capability()

                return GPUInfo(
                    name=gpu_name,
                    vram_mb=vram_mb,
                    cuda_version=cuda_version,
                    compute_capability=compute_capability,
                    is_jetson=is_jetson,
                    is_available=True
                )
    except (subprocess.TimeoutExpired, subprocess.SubprocessError, FileNotFoundError):
        pass

    return None


def get_cuda_version() -> str:
    """Get CUDA version from nvcc or nvidia-smi"""
    try:
        result = subprocess.run(['nvcc', '--version'], capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            for line in result.stdout.split('\n'):
                if 'release' in line.lower():
                    # Extract version like "11.8" from "release 11.8, V11.8.89"
                    parts = line.split('release')[1].split(',')[0].strip()
                    return parts
    except (subprocess.TimeoutExpired, subprocess.SubprocessError, FileNotFoundError):
        pass

    # Fallback: try nvidia-smi
    try:
        result = subprocess.run(['nvidia-smi'], capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            for line in result.stdout.split('\n'):
                if 'CUDA Version' in line:
                    # Extract version from "CUDA Version: 11.8"
                    version = line.split('CUDA Version:')[1].strip().split()[0]
                    return version
    except (subprocess.TimeoutExpired, subprocess.SubprocessError, FileNotFoundError):
        pass

    return "Unknown"


def get_compute_capability() -> str:
    """
    Get CUDA compute capability of GPU.

    Returns:
        Compute capability string (e.g., "7.5", "8.6") or "Unknown"
    """
    try:
        result = subprocess.run(
            ['nvidia-smi', '--query-gpu=compute_cap', '--format=csv,noheader'],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            return result.stdout.strip()
    except (subprocess.TimeoutExpired, subprocess.SubprocessError, FileNotFoundError):
        pass

    return "Unknown"


def check_robot_hardware() -> bool:
    """
    Check if physical robot hardware is connected.

    This checks for:
    - ROS 2 topics from robot driver
    - Connected cameras
    - IMU sensors

    Returns:
        True if robot hardware detected, False otherwise
    """
    try:
        # Check for ROS 2 environment
        if 'ROS_DISTRO' not in os.environ:
            return False

        # Check for robot-specific topics (example: /robot_description, /joint_states)
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True,
            text=True,
            timeout=5
        )

        if result.returncode == 0:
            topics = result.stdout.lower()
            # Look for common robot topics
            robot_indicators = ['/joint_states', '/robot_description', '/imu', '/camera']
            if any(indicator in topics for indicator in robot_indicators):
                return True

    except (subprocess.TimeoutExpired, subprocess.SubprocessError, FileNotFoundError):
        pass

    return False


def detect_tier(override: Optional[str] = None) -> HardwareTier:
    """
    Detect hardware tier based on available resources.

    Priority:
    1. Environment variable ROBOT_TIER (if set)
    2. Manual override parameter
    3. Automatic detection

    Args:
        override: Manual tier override ("simulation", "jetson", "hardware")

    Returns:
        Detected or overridden HardwareTier
    """
    # Check environment variable first
    env_tier = os.environ.get('ROBOT_TIER', '').lower()
    if env_tier in [tier.value for tier in HardwareTier]:
        return HardwareTier(env_tier)

    # Check manual override
    if override:
        override_lower = override.lower()
        if override_lower in [tier.value for tier in HardwareTier]:
            return HardwareTier(override_lower)

    # Automatic detection
    gpu_info = get_gpu_info()

    # No GPU → likely CPU-only fallback (still Tier A with limitations)
    if not gpu_info or not gpu_info.is_available:
        print("Warning: No NVIDIA GPU detected. Module 3 requires GPU for optimal performance.")
        print("Falling back to Tier A (simulation) with CPU-only SLAM alternatives.")
        return HardwareTier.TIER_A

    # Check for physical robot hardware
    if check_robot_hardware():
        return HardwareTier.TIER_C

    # Jetson detection
    if gpu_info.is_jetson:
        return HardwareTier.TIER_B

    # Default: Desktop/laptop with NVIDIA GPU → Tier A (simulation)
    return HardwareTier.TIER_A


def validate_tier_requirements(tier: HardwareTier, gpu_info: Optional[GPUInfo] = None) -> Tuple[bool, str]:
    """
    Validate if current hardware meets requirements for detected tier.

    Args:
        tier: Detected hardware tier
        gpu_info: GPU information (optional, will detect if not provided)

    Returns:
        Tuple of (is_valid, message)
    """
    if not gpu_info:
        gpu_info = get_gpu_info()

    if tier == HardwareTier.TIER_A:
        # Tier A requirements: NVIDIA GPU with 6GB+ VRAM, CUDA 11.8+
        if not gpu_info:
            return False, "Tier A requires NVIDIA GPU (RTX 2060+ recommended)"

        if gpu_info.vram_mb < 6000:
            return False, f"Tier A requires 6GB+ VRAM (detected: {gpu_info.vram_mb} MB)"

        if gpu_info.compute_capability != "Unknown":
            # Require compute capability 7.5+ (Turing or newer)
            cc_major = float(gpu_info.compute_capability.split('.')[0])
            cc_minor = float(gpu_info.compute_capability.split('.')[1]) if '.' in gpu_info.compute_capability else 0
            if cc_major < 7 or (cc_major == 7 and cc_minor < 5):
                return False, f"Tier A requires compute capability 7.5+ (detected: {gpu_info.compute_capability})"

        return True, f"Tier A ready: {gpu_info.name} ({gpu_info.vram_mb} MB VRAM, CUDA {gpu_info.cuda_version})"

    elif tier == HardwareTier.TIER_B:
        # Tier B requirements: Jetson Orin Nano/NX
        if not gpu_info or not gpu_info.is_jetson:
            return False, "Tier B requires NVIDIA Jetson Orin (Nano or NX)"

        if 'orin' not in gpu_info.name.lower():
            return False, f"Tier B requires Jetson Orin (detected: {gpu_info.name})"

        return True, f"Tier B ready: {gpu_info.name} ({gpu_info.vram_mb} MB VRAM)"

    elif tier == HardwareTier.TIER_C:
        # Tier C requirements: All of Tier B + robot hardware
        tier_b_valid, tier_b_msg = validate_tier_requirements(HardwareTier.TIER_B, gpu_info)
        if not tier_b_valid:
            return False, f"Tier C requires Tier B hardware: {tier_b_msg}"

        if not check_robot_hardware():
            return False, "Tier C requires connected robot hardware (check ROS 2 topics)"

        return True, f"Tier C ready: {tier_b_msg} + Robot hardware detected"

    else:
        return False, "Unknown tier"


def main():
    """Main entry point for tier detection CLI"""
    print("=" * 60)
    print("Module 3: Hardware Tier Detection")
    print("=" * 60)

    # Get GPU info
    gpu_info = get_gpu_info()

    if gpu_info:
        print(f"\nGPU Detected:")
        print(f"  Name: {gpu_info.name}")
        print(f"  VRAM: {gpu_info.vram_mb} MB ({gpu_info.vram_mb / 1024:.1f} GB)")
        print(f"  CUDA: {gpu_info.cuda_version}")
        print(f"  Compute Capability: {gpu_info.compute_capability}")
        print(f"  Is Jetson: {gpu_info.is_jetson}")
    else:
        print("\nNo NVIDIA GPU detected")

    # Detect tier
    tier = detect_tier()
    print(f"\nDetected Tier: {tier.value.upper()} ({tier.name})")

    # Tier descriptions
    tier_descriptions = {
        HardwareTier.TIER_A: "Simulation Only - All chapters run in Isaac Sim",
        HardwareTier.TIER_B: "Edge AI - Deploy VSLAM and Nav2 to Jetson Orin",
        HardwareTier.TIER_C: "Physical Robot - Full stack on humanoid hardware"
    }
    print(f"  Description: {tier_descriptions.get(tier, 'Unknown')}")

    # Validate requirements
    is_valid, message = validate_tier_requirements(tier, gpu_info)
    print(f"\nValidation: {'✅ PASS' if is_valid else '❌ FAIL'}")
    print(f"  {message}")

    # Recommendations
    if tier == HardwareTier.TIER_A and gpu_info:
        if gpu_info.vram_mb < 8000:
            print("\n💡 Recommendation: 8GB+ VRAM recommended for optimal VSLAM performance")
        if gpu_info.vram_mb >= 12000:
            print("\n✨ Excellent: Your GPU exceeds recommended specs for Tier A!")

    print("\n" + "=" * 60)
    print("Set tier manually: export ROBOT_TIER=simulation|jetson|hardware")
    print("=" * 60)

    return 0 if is_valid else 1


if __name__ == "__main__":
    sys.exit(main())
