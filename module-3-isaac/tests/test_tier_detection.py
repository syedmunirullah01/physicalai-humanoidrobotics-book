"""
Test suite for tier_detection.py utility

Validates hardware tier detection functionality for Module 3.

Usage:
    pytest tests/test_tier_detection.py
    pytest tests/test_tier_detection.py -v
    pytest tests/test_tier_detection.py::test_gpu_info_detection
"""

import os
import unittest
from unittest.mock import Mock, patch

import pytest

# Import the module under test
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from shared.utils.tier_detection import (
    HardwareTier,
    GPUInfo,
    get_gpu_info,
    get_cuda_version,
    get_compute_capability,
    check_robot_hardware,
    detect_tier,
    validate_tier_requirements
)


class TestGPUDetection(unittest.TestCase):
    """Test GPU detection functionality"""

    @patch('subprocess.run')
    def test_gpu_info_detection_success(self, mock_run):
        """Test successful GPU detection via nvidia-smi"""
        # Mock nvidia-smi output
        mock_run.return_value = Mock(
            returncode=0,
            stdout="NVIDIA GeForce RTX 3060, 12288, 535.129.03\n"
        )

        gpu_info = get_gpu_info()

        assert gpu_info is not None
        assert "RTX 3060" in gpu_info.name
        assert gpu_info.vram_mb == 12288
        assert gpu_info.is_available is True

    @patch('subprocess.run')
    def test_gpu_info_detection_no_gpu(self, mock_run):
        """Test GPU detection when no GPU present"""
        # Mock nvidia-smi failure
        mock_run.side_effect = FileNotFoundError()

        gpu_info = get_gpu_info()

        assert gpu_info is None

    @patch('subprocess.run')
    def test_jetson_detection(self, mock_run):
        """Test detection of Jetson Orin hardware"""
        # Mock Jetson GPU output
        mock_run.return_value = Mock(
            returncode=0,
            stdout="Jetson Orin Nano, 8192, 5.1.2\n"
        )

        with patch('platform.platform', return_value='Linux-5.10.104-tegra-aarch64'):
            gpu_info = get_gpu_info()

            assert gpu_info is not None
            assert gpu_info.is_jetson is True
            assert "Orin" in gpu_info.name


class TestCUDAVersion(unittest.TestCase):
    """Test CUDA version detection"""

    @patch('subprocess.run')
    def test_cuda_version_from_nvcc(self, mock_run):
        """Test CUDA version detection from nvcc"""
        mock_run.return_value = Mock(
            returncode=0,
            stdout="nvcc: NVIDIA (R) Cuda compiler driver\nCopyright (c) 2005-2022 NVIDIA Corporation\nBuilt on Tue_Sep_27_04:45:29_PDT_2022\nCuda compilation tools, release 11.8, V11.8.89\n"
        )

        version = get_cuda_version()

        assert version == "11.8" or "11.8" in version

    @patch('subprocess.run')
    def test_cuda_version_no_cuda(self, mock_run):
        """Test CUDA version when CUDA not installed"""
        mock_run.side_effect = FileNotFoundError()

        version = get_cuda_version()

        assert version == "Unknown"


class TestTierDetection(unittest.TestCase):
    """Test hardware tier detection logic"""

    def test_tier_detection_env_override(self):
        """Test tier detection with environment variable override"""
        with patch.dict(os.environ, {'ROBOT_TIER': 'simulation'}):
            tier = detect_tier()
            assert tier == HardwareTier.TIER_A

        with patch.dict(os.environ, {'ROBOT_TIER': 'jetson'}):
            tier = detect_tier()
            assert tier == HardwareTier.TIER_B

        with patch.dict(os.environ, {'ROBOT_TIER': 'hardware'}):
            tier = detect_tier()
            assert tier == HardwareTier.TIER_C

    def test_tier_detection_manual_override(self):
        """Test tier detection with manual parameter override"""
        tier = detect_tier(override='simulation')
        assert tier == HardwareTier.TIER_A

        tier = detect_tier(override='jetson')
        assert tier == HardwareTier.TIER_B

        tier = detect_tier(override='hardware')
        assert tier == HardwareTier.TIER_C

    @patch('shared.utils.tier_detection.get_gpu_info')
    @patch('shared.utils.tier_detection.check_robot_hardware')
    def test_tier_detection_automatic_simulation(self, mock_robot, mock_gpu):
        """Test automatic detection of Tier A (simulation)"""
        # Mock desktop GPU (not Jetson)
        mock_gpu.return_value = GPUInfo(
            name="NVIDIA GeForce RTX 3060",
            vram_mb=12288,
            cuda_version="11.8",
            compute_capability="8.6",
            is_jetson=False,
            is_available=True
        )
        mock_robot.return_value = False

        tier = detect_tier()

        assert tier == HardwareTier.TIER_A

    @patch('shared.utils.tier_detection.get_gpu_info')
    @patch('shared.utils.tier_detection.check_robot_hardware')
    def test_tier_detection_automatic_jetson(self, mock_robot, mock_gpu):
        """Test automatic detection of Tier B (Jetson)"""
        # Mock Jetson GPU
        mock_gpu.return_value = GPUInfo(
            name="Jetson Orin Nano",
            vram_mb=8192,
            cuda_version="11.4",
            compute_capability="8.7",
            is_jetson=True,
            is_available=True
        )
        mock_robot.return_value = False

        tier = detect_tier()

        assert tier == HardwareTier.TIER_B

    @patch('shared.utils.tier_detection.get_gpu_info')
    @patch('shared.utils.tier_detection.check_robot_hardware')
    def test_tier_detection_automatic_hardware(self, mock_robot, mock_gpu):
        """Test automatic detection of Tier C (physical robot)"""
        # Mock Jetson GPU + robot hardware
        mock_gpu.return_value = GPUInfo(
            name="Jetson Orin Nano",
            vram_mb=8192,
            cuda_version="11.4",
            compute_capability="8.7",
            is_jetson=True,
            is_available=True
        )
        mock_robot.return_value = True

        tier = detect_tier()

        assert tier == HardwareTier.TIER_C

    @patch('shared.utils.tier_detection.get_gpu_info')
    def test_tier_detection_no_gpu_fallback(self, mock_gpu):
        """Test fallback to Tier A when no GPU detected"""
        mock_gpu.return_value = None

        tier = detect_tier()

        assert tier == HardwareTier.TIER_A


class TestTierValidation(unittest.TestCase):
    """Test hardware tier requirement validation"""

    def test_validate_tier_a_sufficient_gpu(self):
        """Test Tier A validation with sufficient GPU"""
        gpu_info = GPUInfo(
            name="NVIDIA GeForce RTX 3060",
            vram_mb=12288,
            cuda_version="11.8",
            compute_capability="8.6",
            is_jetson=False,
            is_available=True
        )

        is_valid, message = validate_tier_requirements(HardwareTier.TIER_A, gpu_info)

        assert is_valid is True
        assert "ready" in message.lower()

    def test_validate_tier_a_insufficient_vram(self):
        """Test Tier A validation with insufficient VRAM"""
        gpu_info = GPUInfo(
            name="NVIDIA GeForce GTX 1050",
            vram_mb=4096,  # < 6GB required
            cuda_version="11.8",
            compute_capability="6.1",
            is_jetson=False,
            is_available=True
        )

        is_valid, message = validate_tier_requirements(HardwareTier.TIER_A, gpu_info)

        assert is_valid is False
        assert "6GB+" in message or "vram" in message.lower()

    def test_validate_tier_a_old_compute_capability(self):
        """Test Tier A validation with old compute capability"""
        gpu_info = GPUInfo(
            name="NVIDIA GeForce GTX 980",
            vram_mb=8192,
            cuda_version="11.8",
            compute_capability="5.2",  # < 7.5 required
            is_jetson=False,
            is_available=True
        )

        is_valid, message = validate_tier_requirements(HardwareTier.TIER_A, gpu_info)

        assert is_valid is False
        assert "compute capability" in message.lower()

    def test_validate_tier_b_jetson_orin(self):
        """Test Tier B validation with Jetson Orin"""
        gpu_info = GPUInfo(
            name="Jetson Orin Nano",
            vram_mb=8192,
            cuda_version="11.4",
            compute_capability="8.7",
            is_jetson=True,
            is_available=True
        )

        is_valid, message = validate_tier_requirements(HardwareTier.TIER_B, gpu_info)

        assert is_valid is True
        assert "ready" in message.lower()

    def test_validate_tier_b_not_jetson(self):
        """Test Tier B validation with non-Jetson GPU"""
        gpu_info = GPUInfo(
            name="NVIDIA GeForce RTX 3060",
            vram_mb=12288,
            cuda_version="11.8",
            compute_capability="8.6",
            is_jetson=False,
            is_available=True
        )

        is_valid, message = validate_tier_requirements(HardwareTier.TIER_B, gpu_info)

        assert is_valid is False
        assert "jetson" in message.lower()


class TestRobotHardwareDetection(unittest.TestCase):
    """Test physical robot hardware detection"""

    @patch('subprocess.run')
    def test_robot_hardware_detected(self, mock_run):
        """Test detection of robot hardware via ROS 2 topics"""
        mock_run.return_value = Mock(
            returncode=0,
            stdout="/joint_states\n/robot_description\n/camera/image_raw\n"
        )

        with patch.dict(os.environ, {'ROS_DISTRO': 'humble'}):
            has_robot = check_robot_hardware()

            assert has_robot is True

    @patch('subprocess.run')
    def test_robot_hardware_not_detected(self, mock_run):
        """Test when robot hardware is not detected"""
        mock_run.return_value = Mock(
            returncode=0,
            stdout="/rosout\n/parameter_events\n"
        )

        with patch.dict(os.environ, {'ROS_DISTRO': 'humble'}):
            has_robot = check_robot_hardware()

            assert has_robot is False

    def test_robot_hardware_no_ros(self):
        """Test robot detection when ROS 2 not available"""
        with patch.dict(os.environ, {}, clear=True):
            has_robot = check_robot_hardware()

            assert has_robot is False


class TestComputeCapability(unittest.TestCase):
    """Test CUDA compute capability detection"""

    @patch('subprocess.run')
    def test_compute_capability_detection(self, mock_run):
        """Test successful compute capability detection"""
        mock_run.return_value = Mock(
            returncode=0,
            stdout="8.6\n"
        )

        cc = get_compute_capability()

        assert cc == "8.6"

    @patch('subprocess.run')
    def test_compute_capability_unknown(self, mock_run):
        """Test compute capability when detection fails"""
        mock_run.side_effect = FileNotFoundError()

        cc = get_compute_capability()

        assert cc == "Unknown"


# Integration tests
class TestTierDetectionIntegration(unittest.TestCase):
    """Integration tests for complete tier detection workflow"""

    @patch('shared.utils.tier_detection.get_gpu_info')
    @patch('shared.utils.tier_detection.check_robot_hardware')
    def test_full_tier_a_workflow(self, mock_robot, mock_gpu):
        """Test complete Tier A detection and validation workflow"""
        # Setup: Desktop GPU, no robot
        mock_gpu.return_value = GPUInfo(
            name="NVIDIA GeForce RTX 4070",
            vram_mb=12288,
            cuda_version="12.1",
            compute_capability="8.9",
            is_jetson=False,
            is_available=True
        )
        mock_robot.return_value = False

        # Detect tier
        tier = detect_tier()
        assert tier == HardwareTier.TIER_A

        # Validate requirements
        gpu_info = get_gpu_info()
        is_valid, message = validate_tier_requirements(tier, gpu_info)

        assert is_valid is True
        assert "4070" in message

    @patch('shared.utils.tier_detection.get_gpu_info')
    @patch('shared.utils.tier_detection.check_robot_hardware')
    def test_full_tier_c_workflow(self, mock_robot, mock_gpu):
        """Test complete Tier C detection and validation workflow"""
        # Setup: Jetson + robot hardware
        mock_gpu.return_value = GPUInfo(
            name="Jetson Orin NX",
            vram_mb=16384,
            cuda_version="11.4",
            compute_capability="8.7",
            is_jetson=True,
            is_available=True
        )
        mock_robot.return_value = True

        # Detect tier
        tier = detect_tier()
        assert tier == HardwareTier.TIER_C

        # Validate requirements
        gpu_info = get_gpu_info()
        is_valid, message = validate_tier_requirements(tier, gpu_info)

        assert is_valid is True


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
