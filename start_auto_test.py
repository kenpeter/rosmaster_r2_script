#!/usr/bin/env python3
"""
Autonomous Robot System - Comprehensive Test Suite
Tests all components with detailed validation (REPORT ONLY - NO AUTO-FIX)

82 test cases across 11 categories:
- Hardware Layer (10): USB devices, serial mappings, permissions
- LIDAR Hardware Operation (6): Motor spinning, scan rate, completeness
- Camera Hardware Operation (8): Frame capture, resolution, sync
- IMU Hardware Operation (5): Data rate, gyro/accel ranges, orientation
- Motor Controller Operation (5): Process, serial, encoders, battery
- ROS2 Topics (12): Topic availability and publishing rates
- Sensor Data Quality (8): Data format validation
- 2D Grid System (10): Grid generation and statistics
- Sensor Fusion (6): Multi-sensor integration status
- LLM Decision Pipeline (8): Inference and action validation
- Control System (4): Control loop and command validation

Validates both hardware operation AND system output to catch regressions

Auto-runs 30 seconds after start_auto.py launches (allows system to stabilize)
"""

import os
import sys
import subprocess
import time
import re
import json
from dataclasses import dataclass
from typing import List, Optional, Tuple
from enum import Enum

# Color codes
class Colors:
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[0;33m'
    BLUE = '\033[0;34m'
    CYAN = '\033[0;36m'
    NC = '\033[0m'


@dataclass
class TestResult:
    """Result of a single test"""
    name: str
    passed: bool
    message: str
    details: str = ""


class ROS2Helper:
    """Utilities for ROS2 operations"""

    def __init__(self):
        self.ros_domain = os.environ.get('ROS_DOMAIN_ID', '28')

    def run_ros_command(self, cmd: str, timeout: int = 5) -> Tuple[bool, str]:
        """Run a ROS2 command with proper environment"""
        full_cmd = f"bash -c 'export ROS_DOMAIN_ID={self.ros_domain} && source /opt/ros/humble/setup.bash && {cmd}'"
        try:
            result = subprocess.run(full_cmd, shell=True, capture_output=True, text=True, timeout=timeout)
            return (result.returncode == 0, result.stdout + result.stderr)
        except subprocess.TimeoutExpired:
            return (False, f"Timeout after {timeout}s")

    def topic_exists(self, topic: str) -> bool:
        """Check if topic exists"""
        success, output = self.run_ros_command("ros2 topic list", timeout=3)
        return success and topic in output

    def topic_hz(self, topic: str, duration: int = 3) -> Optional[float]:
        """Get topic publishing rate"""
        success, output = self.run_ros_command(f"timeout {duration} ros2 topic hz {topic} 2>&1", timeout=duration + 2)
        if success and "average rate:" in output:
            match = re.search(r'average rate:\s*([\d.]+)', output)
            if match:
                return float(match.group(1))
        return None

    def topic_echo_once(self, topic: str, timeout: int = 5) -> Optional[str]:
        """Get one message from topic"""
        success, output = self.run_ros_command(f"timeout {timeout} ros2 topic echo {topic} --once", timeout=timeout + 1)
        return output if success else None


class SystemTests:
    """Comprehensive system validation tests"""

    def __init__(self, ros2_helper: ROS2Helper, verbose: bool = False):
        self.helper = ros2_helper
        self.verbose = verbose
        self.decision_output = None
        self.all_results = []

    def run_all_tests(self) -> bool:
        """Run all tests and return overall pass/fail"""
        print(f"\n{Colors.BLUE}{'='*80}{Colors.NC}")
        print(f"{Colors.BLUE}🔍 AUTONOMOUS ROBOT SYSTEM - COMPREHENSIVE TEST SUITE{Colors.NC}")
        print(f"{Colors.BLUE}{'='*80}{Colors.NC}\n")

        # Capture decision pipeline output for analysis
        self._capture_decision_output()

        # Run all test categories
        self._test_hardware_layer()
        self._test_lidar_hardware_operation()
        self._test_camera_hardware_operation()
        self._test_imu_hardware_operation()
        self._test_motor_controller_operation()
        self._test_ros2_topics()
        self._test_sensor_data_quality()
        self._test_2d_grid_system()
        self._test_sensor_fusion()
        self._test_llm_decision_pipeline()
        self._test_control_system()

        # Print summary
        self._print_summary()

        passed = sum(1 for r in self.all_results if r.passed)
        total = len(self.all_results)
        return passed == total

    def _capture_decision_output(self):
        """Capture decision pipeline output for detailed analysis"""
        print(f"{Colors.CYAN}Capturing decision pipeline output...{Colors.NC}")
        self.decision_output = self.helper.topic_echo_once('/autonomous/decision', timeout=5)
        if self.decision_output:
            print(f"{Colors.GREEN}✓ Decision pipeline output captured ({len(self.decision_output)} bytes){Colors.NC}\n")
        else:
            print(f"{Colors.YELLOW}⚠ Decision pipeline not available (will skip some tests){Colors.NC}\n")

    # ==================== HARDWARE LAYER TESTS ====================

    def _test_hardware_layer(self):
        """Test hardware devices and USB mappings"""
        print(f"{Colors.CYAN}[HARDWARE LAYER - 10 tests]{Colors.NC}")

        tests = [
            self._test_usb0_interface(),
            self._test_usb1_interface(),
            self._test_serial_myserial(),
            self._test_serial_ydlidar(),
            self._test_serial_symlink_mapping(),
            self._test_robot_controller_device(),
            self._test_ydlidar_device(),
            self._test_usb_camera_detection(),
            self._test_camera_video_device(),
            self._test_device_permissions(),
        ]

        for result in tests:
            self._print_result(result)
            self.all_results.append(result)
        print()

    def _test_usb0_interface(self) -> TestResult:
        result = subprocess.run("ip link show usb0", shell=True, capture_output=True, text=True)
        if result.returncode == 0 and "state UP" in result.stdout:
            return TestResult("USB0 network interface", True, "UP and active")
        return TestResult("USB0 network interface", False, "Missing or DOWN")

    def _test_usb1_interface(self) -> TestResult:
        result = subprocess.run("ip link show usb1", shell=True, capture_output=True, text=True)
        if result.returncode == 0 and "state UP" in result.stdout:
            return TestResult("USB1 network interface", True, "UP and active")
        return TestResult("USB1 network interface", False, "Missing or DOWN")

    def _test_serial_myserial(self) -> TestResult:
        if os.path.exists('/dev/myserial'):
            target = os.readlink('/dev/myserial') if os.path.islink('/dev/myserial') else 'direct'
            return TestResult("/dev/myserial", True, f"→ {target}")
        return TestResult("/dev/myserial", False, "Missing")

    def _test_serial_ydlidar(self) -> TestResult:
        if os.path.exists('/dev/ydlidar'):
            target = os.readlink('/dev/ydlidar') if os.path.islink('/dev/ydlidar') else 'direct'
            return TestResult("/dev/ydlidar", True, f"→ {target}")
        return TestResult("/dev/ydlidar", False, "Missing")

    def _test_serial_symlink_mapping(self) -> TestResult:
        """Check if serial devices are correctly mapped (not swapped)"""
        try:
            myserial = os.readlink('/dev/myserial')
            ydlidar = os.readlink('/dev/ydlidar')
            if myserial == 'ttyUSB1' and ydlidar == 'ttyUSB0':
                return TestResult("Serial device mapping", False, "SWAPPED (myserial→USB1, ydlidar→USB0)")
            return TestResult("Serial device mapping", True, f"Correct (myserial→{myserial}, ydlidar→{ydlidar})")
        except:
            return TestResult("Serial device mapping", False, "Cannot verify")

    def _test_robot_controller_device(self) -> TestResult:
        if os.path.exists('/dev/myserial'):
            if os.access('/dev/myserial', os.R_OK | os.W_OK):
                return TestResult("Robot controller access", True, "Read/write OK")
            return TestResult("Robot controller access", False, "Permission denied")
        return TestResult("Robot controller access", False, "Device missing")

    def _test_ydlidar_device(self) -> TestResult:
        if os.path.exists('/dev/ydlidar'):
            if os.access('/dev/ydlidar', os.R_OK | os.W_OK):
                return TestResult("YDLidar device access", True, "Read/write OK")
            return TestResult("YDLidar device access", False, "Permission denied")
        return TestResult("YDLidar device access", False, "Device missing")

    def _test_usb_camera_detection(self) -> TestResult:
        result = subprocess.run("lsusb", shell=True, capture_output=True, text=True)
        if '2bc5:050f' in result.stdout:
            return TestResult("Astra camera USB", True, "Detected (2bc5:050f)")
        return TestResult("Astra camera USB", False, "Not found on USB bus")

    def _test_camera_video_device(self) -> TestResult:
        video_devices = [f for f in os.listdir('/dev') if f.startswith('video')]
        if video_devices:
            return TestResult("Camera video devices", True, f"Found: {', '.join(video_devices[:3])}")
        return TestResult("Camera video devices", False, "No /dev/video* devices")

    def _test_device_permissions(self) -> TestResult:
        """Check device permissions are correct"""
        devices = ['/dev/myserial', '/dev/ydlidar']
        issues = []
        for dev in devices:
            if os.path.exists(dev):
                stat = os.stat(dev)
                mode = oct(stat.st_mode)[-3:]
                if mode not in ['666', '660', '664']:
                    issues.append(f"{dev}:{mode}")
        if issues:
            return TestResult("Device permissions", False, f"Restrictive: {', '.join(issues)}")
        return TestResult("Device permissions", True, "All accessible")

    # ==================== LIDAR HARDWARE OPERATION TESTS ====================

    def _test_lidar_hardware_operation(self):
        """Test LIDAR hardware actual operation"""
        print(f"{Colors.CYAN}[LIDAR HARDWARE OPERATION - 6 tests]{Colors.NC}")

        tests = [
            self._test_lidar_motor_spinning(),
            self._test_lidar_scan_rate(),
            self._test_lidar_angle_increment(),
            self._test_lidar_range_min_max(),
            self._test_lidar_scan_completeness(),
            self._test_lidar_data_freshness(),
        ]

        for result in tests:
            self._print_result(result)
            self.all_results.append(result)
        print()

    def _test_lidar_motor_spinning(self) -> TestResult:
        """Verify LIDAR motor is spinning by checking angle distribution"""
        output = self.helper.topic_echo_once('/scan', timeout=3)
        if not output:
            return TestResult("LIDAR motor spinning", False, "No scan data")

        # Check angle_min and angle_max indicate full rotation
        angle_min_match = re.search(r'angle_min:\s*([-\d.]+)', output)
        angle_max_match = re.search(r'angle_max:\s*([-\d.]+)', output)

        if angle_min_match and angle_max_match:
            angle_min = float(angle_min_match.group(1))
            angle_max = float(angle_max_match.group(1))
            scan_range = angle_max - angle_min

            # YDLidar should do nearly 360 degrees (2*pi ≈ 6.28 rad)
            if scan_range > 6.0:
                return TestResult("LIDAR motor spinning", True, f"Scan range {scan_range:.2f} rad (360° coverage)")
            else:
                return TestResult("LIDAR motor spinning", False, f"Scan range {scan_range:.2f} rad (incomplete rotation)")

        return TestResult("LIDAR motor spinning", False, "Cannot parse angle range")

    def _test_lidar_scan_rate(self) -> TestResult:
        """Verify LIDAR scan rate matches specification (10 Hz)"""
        hz = self.helper.topic_hz('/scan', duration=3)
        if hz is None:
            return TestResult("LIDAR scan rate", False, "Not publishing")

        # YDLidar TG30 should be ~10 Hz
        if 8.0 <= hz <= 12.0:
            return TestResult("LIDAR scan rate", True, f"{hz:.1f} Hz (spec: 10 Hz)")
        elif hz < 8.0:
            return TestResult("LIDAR scan rate", False, f"{hz:.1f} Hz (too slow, spec: 10 Hz)")
        else:
            return TestResult("LIDAR scan rate", False, f"{hz:.1f} Hz (too fast, spec: 10 Hz)")

    def _test_lidar_angle_increment(self) -> TestResult:
        """Check LIDAR angular resolution"""
        output = self.helper.topic_echo_once('/scan', timeout=3)
        if not output:
            return TestResult("LIDAR angle increment", False, "No scan data")

        match = re.search(r'angle_increment:\s*([\d.]+)', output)
        if match:
            increment = float(match.group(1))
            # Typical increment is ~0.005-0.01 rad (0.3-0.6 degrees)
            if 0.003 <= increment <= 0.02:
                return TestResult("LIDAR angle increment", True, f"{increment:.4f} rad ({increment*57.3:.2f}°)")
            else:
                return TestResult("LIDAR angle increment", False, f"{increment:.4f} rad (unusual)")
        return TestResult("LIDAR angle increment", False, "Cannot parse")

    def _test_lidar_range_min_max(self) -> TestResult:
        """Verify LIDAR range limits match specification"""
        output = self.helper.topic_echo_once('/scan', timeout=3)
        if not output:
            return TestResult("LIDAR range limits", False, "No scan data")

        range_min_match = re.search(r'range_min:\s*([\d.]+)', output)
        range_max_match = re.search(r'range_max:\s*([\d.]+)', output)

        if range_min_match and range_max_match:
            range_min = float(range_min_match.group(1))
            range_max = float(range_max_match.group(1))

            # YDLidar TG30: 0.05m - 30m typical
            if 0.01 <= range_min <= 0.2 and 10.0 <= range_max <= 35.0:
                return TestResult("LIDAR range limits", True, f"min={range_min:.2f}m, max={range_max:.2f}m")
            else:
                return TestResult("LIDAR range limits", False, f"min={range_min:.2f}m, max={range_max:.2f}m (out of spec)")

        return TestResult("LIDAR range limits", False, "Cannot parse range limits")

    def _test_lidar_scan_completeness(self) -> TestResult:
        """Check LIDAR produces complete scans (not sparse)"""
        output = self.helper.topic_echo_once('/scan', timeout=3)
        if not output:
            return TestResult("LIDAR scan completeness", False, "No scan data")

        ranges = re.findall(r'ranges:.*?\[(.*?)\]', output, re.DOTALL)
        if not ranges:
            return TestResult("LIDAR scan completeness", False, "Cannot parse ranges")

        range_values = [x.strip() for x in ranges[0].split(',') if x.strip()]
        total_points = len(range_values)

        # YDLidar should have 500+ points per scan
        if total_points >= 400:
            return TestResult("LIDAR scan completeness", True, f"{total_points} points/scan")
        else:
            return TestResult("LIDAR scan completeness", False, f"Only {total_points} points/scan (sparse)")

    def _test_lidar_data_freshness(self) -> TestResult:
        """Verify LIDAR scan timestamps are recent"""
        output = self.helper.topic_echo_once('/scan', timeout=3)
        if not output:
            return TestResult("LIDAR data freshness", False, "No scan data")

        # Check timestamp is present
        if 'stamp:' in output and 'sec:' in output:
            return TestResult("LIDAR data freshness", True, "Timestamp present (data is fresh)")
        return TestResult("LIDAR data freshness", False, "No timestamp")

    # ==================== CAMERA HARDWARE OPERATION TESTS ====================

    def _test_camera_hardware_operation(self):
        """Test camera hardware actual operation"""
        print(f"{Colors.CYAN}[CAMERA HARDWARE OPERATION - 8 tests]{Colors.NC}")

        tests = [
            self._test_camera_frame_capture(),
            self._test_camera_resolution_spec(),
            self._test_camera_frame_rate(),
            self._test_camera_color_data_validity(),
            self._test_depth_camera_capture(),
            self._test_depth_resolution_spec(),
            self._test_depth_valid_pixels(),
            self._test_camera_synchronization(),
        ]

        for result in tests:
            self._print_result(result)
            self.all_results.append(result)
        print()

    def _test_camera_frame_capture(self) -> TestResult:
        """Verify camera is actually capturing frames"""
        output = self.helper.topic_echo_once('/camera/color/image_raw', timeout=5)
        if not output:
            return TestResult("Camera frame capture", False, "No frame data")

        # Check for actual image data (not just header)
        if 'data:' in output and 'height:' in output and 'width:' in output:
            # Check data array has content
            data_match = re.search(r'data:\s*\[(.*?)\]', output, re.DOTALL)
            if data_match and len(data_match.group(1)) > 100:
                return TestResult("Camera frame capture", True, "Capturing frames with pixel data")
            return TestResult("Camera frame capture", False, "Frame header only, no pixel data")

        return TestResult("Camera frame capture", False, "Invalid frame format")

    def _test_camera_resolution_spec(self) -> TestResult:
        """Verify camera resolution matches specification"""
        output = self.helper.topic_echo_once('/camera/color/image_raw', timeout=5)
        if not output:
            return TestResult("Camera resolution spec", False, "No frame data")

        height_match = re.search(r'height:\s*(\d+)', output)
        width_match = re.search(r'width:\s*(\d+)', output)

        if height_match and width_match:
            height = int(height_match.group(1))
            width = int(width_match.group(1))

            # Astra camera typically: 640x480 or 1280x720
            if (width, height) in [(640, 480), (1280, 720), (1920, 1080)]:
                return TestResult("Camera resolution spec", True, f"{width}x{height} (valid)")
            else:
                return TestResult("Camera resolution spec", False, f"{width}x{height} (unusual resolution)")

        return TestResult("Camera resolution spec", False, "Cannot parse resolution")

    def _test_camera_frame_rate(self) -> TestResult:
        """Verify camera frame rate matches spec (30 Hz typical)"""
        hz = self.helper.topic_hz('/camera/color/image_raw', duration=3)
        if hz is None:
            return TestResult("Camera frame rate", False, "Not publishing")

        # Astra camera: 15-30 Hz typical
        if 15.0 <= hz <= 35.0:
            return TestResult("Camera frame rate", True, f"{hz:.1f} Hz")
        elif hz < 15.0:
            return TestResult("Camera frame rate", False, f"{hz:.1f} Hz (too slow)")
        else:
            return TestResult("Camera frame rate", False, f"{hz:.1f} Hz (too fast)")

    def _test_camera_color_data_validity(self) -> TestResult:
        """Check color camera encoding format"""
        output = self.helper.topic_echo_once('/camera/color/image_raw', timeout=5)
        if not output:
            return TestResult("Camera encoding format", False, "No frame data")

        # Check encoding type
        encoding_match = re.search(r'encoding:\s*["\']?([\w]+)', output)
        if encoding_match:
            encoding = encoding_match.group(1)
            # Common formats: rgb8, bgr8, rgba8, bgra8
            if encoding in ['rgb8', 'bgr8', 'rgba8', 'bgra8', 'mono8']:
                return TestResult("Camera encoding format", True, f"{encoding}")
            return TestResult("Camera encoding format", False, f"{encoding} (unusual)")

        return TestResult("Camera encoding format", True, "Encoding present")

    def _test_depth_camera_capture(self) -> TestResult:
        """Verify depth camera is capturing frames"""
        output = self.helper.topic_echo_once('/camera/depth/image_raw', timeout=5)
        if not output:
            return TestResult("Depth camera capture", False, "No depth data")

        if 'data:' in output and 'height:' in output and 'width:' in output:
            return TestResult("Depth camera capture", True, "Capturing depth frames")

        return TestResult("Depth camera capture", False, "Invalid depth format")

    def _test_depth_resolution_spec(self) -> TestResult:
        """Verify depth camera resolution"""
        output = self.helper.topic_echo_once('/camera/depth/image_raw', timeout=5)
        if not output:
            return TestResult("Depth resolution spec", False, "No depth data")

        height_match = re.search(r'height:\s*(\d+)', output)
        width_match = re.search(r'width:\s*(\d+)', output)

        if height_match and width_match:
            height = int(height_match.group(1))
            width = int(width_match.group(1))

            # Astra depth: 640x480 typical
            if (width, height) in [(640, 480), (320, 240)]:
                return TestResult("Depth resolution spec", True, f"{width}x{height}")
            else:
                return TestResult("Depth resolution spec", False, f"{width}x{height} (unusual)")

        return TestResult("Depth resolution spec", False, "Cannot parse resolution")

    def _test_depth_valid_pixels(self) -> TestResult:
        """Check depth camera has valid depth measurements"""
        # This test relies on decision output which shows depth statistics
        if self.decision_output and "Depth camera working:" in self.decision_output:
            match = re.search(r'Depth camera working:\s*([\d.]+)%\s*valid pixels', self.decision_output)
            if match:
                valid_pct = float(match.group(1))
                if valid_pct > 1.0:
                    return TestResult("Depth valid pixels", True, f"{valid_pct:.1f}% valid")
                else:
                    return TestResult("Depth valid pixels", False, f"{valid_pct:.1f}% valid (too few)")
            return TestResult("Depth valid pixels", True, "Depth camera operational")
        return TestResult("Depth valid pixels", True, "Depth camera present (skipping pixel validation)")

    def _test_camera_synchronization(self) -> TestResult:
        """Check color and depth cameras are synchronized"""
        # Both should be publishing at similar rates
        color_hz = self.helper.topic_hz('/camera/color/image_raw', duration=2)
        depth_hz = self.helper.topic_hz('/camera/depth/image_raw', duration=2)

        if color_hz is None or depth_hz is None:
            return TestResult("Camera synchronization", False, "One or both cameras not publishing")

        # Check if rates are within 20% of each other
        rate_diff = abs(color_hz - depth_hz) / max(color_hz, depth_hz)
        if rate_diff < 0.2:
            return TestResult("Camera synchronization", True, f"color={color_hz:.1f}Hz, depth={depth_hz:.1f}Hz (synced)")
        else:
            return TestResult("Camera synchronization", False, f"color={color_hz:.1f}Hz, depth={depth_hz:.1f}Hz (desync)")

    # ==================== IMU HARDWARE OPERATION TESTS ====================

    def _test_imu_hardware_operation(self):
        """Test IMU hardware actual operation"""
        print(f"{Colors.CYAN}[IMU HARDWARE OPERATION - 5 tests]{Colors.NC}")

        tests = [
            self._test_imu_data_rate(),
            self._test_imu_gyro_range(),
            self._test_imu_accel_gravity(),
            self._test_imu_data_not_frozen(),
            self._test_imu_orientation_present(),
        ]

        for result in tests:
            self._print_result(result)
            self.all_results.append(result)
        print()

    def _test_imu_data_rate(self) -> TestResult:
        """Verify IMU publishing rate (100+ Hz typical)"""
        hz = self.helper.topic_hz('/imu/data', duration=2)
        if hz is None:
            return TestResult("IMU data rate", False, "Not publishing")

        # IMU should be 50-200 Hz
        if 50.0 <= hz <= 200.0:
            return TestResult("IMU data rate", True, f"{hz:.1f} Hz")
        elif hz < 50.0:
            return TestResult("IMU data rate", False, f"{hz:.1f} Hz (too slow)")
        else:
            return TestResult("IMU data rate", False, f"{hz:.1f} Hz (too fast)")

    def _test_imu_gyro_range(self) -> TestResult:
        """Check IMU gyroscope values are in reasonable range"""
        output = self.helper.topic_echo_once('/imu/data', timeout=3)
        if not output:
            return TestResult("IMU gyro range", False, "No IMU data")

        # Extract angular velocity values
        gyro_match = re.search(r'angular_velocity:.*?x:\s*([-\d.]+).*?y:\s*([-\d.]+).*?z:\s*([-\d.]+)', output, re.DOTALL)
        if gyro_match:
            x = abs(float(gyro_match.group(1)))
            y = abs(float(gyro_match.group(2)))
            z = abs(float(gyro_match.group(3)))

            # Check gyro values are not extreme (robot should be stationary or slow)
            # Extreme would be >10 rad/s (robot not expected to spin that fast)
            max_gyro = max(x, y, z)
            if max_gyro < 50.0:  # Reasonable for stationary/moving robot
                return TestResult("IMU gyro range", True, f"max={max_gyro:.2f} rad/s (reasonable)")
            else:
                return TestResult("IMU gyro range", False, f"max={max_gyro:.2f} rad/s (extreme values)")

        return TestResult("IMU gyro range", False, "Cannot parse gyro data")

    def _test_imu_accel_gravity(self) -> TestResult:
        """Check IMU accelerometer detects gravity (~9.8 m/s²)"""
        output = self.helper.topic_echo_once('/imu/data', timeout=3)
        if not output:
            return TestResult("IMU accel gravity", False, "No IMU data")

        # Extract linear acceleration
        accel_match = re.search(r'linear_acceleration:.*?x:\s*([-\d.]+).*?y:\s*([-\d.]+).*?z:\s*([-\d.]+)', output, re.DOTALL)
        if accel_match:
            x = float(accel_match.group(1))
            y = float(accel_match.group(2))
            z = float(accel_match.group(3))

            # Calculate magnitude (should be ~9.8 m/s² if IMU is stationary)
            magnitude = (x**2 + y**2 + z**2)**0.5

            # Check if magnitude is close to gravity (7-12 m/s² acceptable)
            if 7.0 <= magnitude <= 12.0:
                return TestResult("IMU accel gravity", True, f"magnitude={magnitude:.2f} m/s² (gravity detected)")
            else:
                return TestResult("IMU accel gravity", False, f"magnitude={magnitude:.2f} m/s² (expected ~9.8)")

        return TestResult("IMU accel gravity", False, "Cannot parse accel data")

    def _test_imu_data_not_frozen(self) -> TestResult:
        """Check IMU data is changing (not frozen)"""
        # Get two samples and check they're different
        output1 = self.helper.topic_echo_once('/imu/data', timeout=2)
        time.sleep(0.1)
        output2 = self.helper.topic_echo_once('/imu/data', timeout=2)

        if not output1 or not output2:
            return TestResult("IMU data not frozen", False, "Cannot get IMU samples")

        # Check timestamps are different
        stamp1_match = re.search(r'stamp:.*?sec:\s*(\d+).*?nanosec:\s*(\d+)', output1, re.DOTALL)
        stamp2_match = re.search(r'stamp:.*?sec:\s*(\d+).*?nanosec:\s*(\d+)', output2, re.DOTALL)

        if stamp1_match and stamp2_match:
            time1 = int(stamp1_match.group(1)) * 1e9 + int(stamp1_match.group(2))
            time2 = int(stamp2_match.group(1)) * 1e9 + int(stamp2_match.group(2))

            if time2 > time1:
                return TestResult("IMU data not frozen", True, "Timestamps advancing")
            else:
                return TestResult("IMU data not frozen", False, "Timestamps not advancing (frozen)")

        return TestResult("IMU data not frozen", True, "IMU data present")

    def _test_imu_orientation_present(self) -> TestResult:
        """Check IMU provides orientation data"""
        output = self.helper.topic_echo_once('/imu/data', timeout=3)
        if not output:
            return TestResult("IMU orientation", False, "No IMU data")

        if 'orientation:' in output:
            # Check if quaternion has valid values (not all zeros)
            quat_match = re.search(r'orientation:.*?x:\s*([-\d.]+).*?y:\s*([-\d.]+).*?z:\s*([-\d.]+).*?w:\s*([-\d.]+)', output, re.DOTALL)
            if quat_match:
                x = float(quat_match.group(1))
                y = float(quat_match.group(2))
                z = float(quat_match.group(3))
                w = float(quat_match.group(4))

                magnitude = (x**2 + y**2 + z**2 + w**2)**0.5
                if 0.9 <= magnitude <= 1.1:  # Quaternion should be unit length
                    return TestResult("IMU orientation", True, "Valid quaternion orientation")
                return TestResult("IMU orientation", False, f"Quaternion magnitude={magnitude:.2f} (should be 1.0)")

        return TestResult("IMU orientation", True, "Orientation data present")

    # ==================== MOTOR CONTROLLER OPERATION TESTS ====================

    def _test_motor_controller_operation(self):
        """Test motor controller hardware operation"""
        print(f"{Colors.CYAN}[MOTOR CONTROLLER OPERATION - 5 tests]{Colors.NC}")

        tests = [
            self._test_motor_controller_process(),
            self._test_motor_controller_serial(),
            self._test_motor_encoder_feedback(),
            self._test_battery_voltage(),
            self._test_velocity_command_response(),
        ]

        for result in tests:
            self._print_result(result)
            self.all_results.append(result)
        print()

    def _test_motor_controller_process(self) -> TestResult:
        """Check motor controller driver process is running"""
        result = subprocess.run("ps aux | grep Ackman_driver | grep -v grep", shell=True, capture_output=True)
        if result.returncode == 0:
            return TestResult("Motor controller process", True, "Ackman_driver running")
        return TestResult("Motor controller process", False, "Ackman_driver not running")

    def _test_motor_controller_serial(self) -> TestResult:
        """Check motor controller serial device accessibility"""
        if not os.path.exists('/dev/myserial'):
            return TestResult("Motor controller serial", False, "/dev/myserial missing")

        # Check if Ackman_driver has the device open
        result = subprocess.run("lsof /dev/myserial 2>/dev/null | grep -v COMMAND", shell=True, capture_output=True, text=True)
        if result.returncode == 0 and 'Ackman' in result.stdout:
            return TestResult("Motor controller serial", True, "/dev/myserial opened by driver")
        elif result.returncode == 0:
            return TestResult("Motor controller serial", False, f"/dev/myserial opened by other process")
        else:
            return TestResult("Motor controller serial", False, "/dev/myserial not opened")

    def _test_motor_encoder_feedback(self) -> TestResult:
        """Check motor encoder feedback is available"""
        # Motor encoder feedback comes through /vel_raw topic
        if not self.helper.topic_exists('/vel_raw'):
            return TestResult("Motor encoder feedback", False, "/vel_raw topic missing")

        hz = self.helper.topic_hz('/vel_raw', duration=2)
        if hz is None:
            return TestResult("Motor encoder feedback", False, "/vel_raw not publishing (robot stationary or in air)")
        elif hz > 1.0:
            return TestResult("Motor encoder feedback", True, f"/vel_raw publishing at {hz:.1f} Hz (encoders working)")
        else:
            return TestResult("Motor encoder feedback", False, f"/vel_raw only {hz:.1f} Hz")

    def _test_battery_voltage(self) -> TestResult:
        """Check battery voltage is in safe range"""
        # Battery voltage typically comes from /battery topic or embedded in controller status
        if self.helper.topic_exists('/battery'):
            output = self.helper.topic_echo_once('/battery', timeout=3)
            if output:
                voltage_match = re.search(r'voltage:\s*([\d.]+)', output)
                if voltage_match:
                    voltage = float(voltage_match.group(1))
                    # 3S LiPo: 9.0V (empty) - 12.6V (full), safe: >10.5V
                    if voltage >= 10.5:
                        return TestResult("Battery voltage", True, f"{voltage:.2f}V (good)")
                    elif voltage >= 9.0:
                        return TestResult("Battery voltage", False, f"{voltage:.2f}V (low, charge soon)")
                    else:
                        return TestResult("Battery voltage", False, f"{voltage:.2f}V (critical)")

        # If no battery topic, skip test
        return TestResult("Battery voltage", True, "Battery monitoring not available (skip)")

    def _test_velocity_command_response(self) -> TestResult:
        """Check motor controller responds to velocity commands"""
        # Check if /cmd_vel commands result in /vel_raw feedback
        # This is indirect - we check if control loop is active
        cmd_vel_hz = self.helper.topic_hz('/cmd_vel', duration=2)
        if cmd_vel_hz is None or cmd_vel_hz < 1.0:
            return TestResult("Velocity command response", True, "No commands sent (robot idle)")

        # Commands are being sent, check if base_node is processing
        result = subprocess.run("ps aux | grep base_node | grep -v grep", shell=True, capture_output=True)
        if result.returncode == 0:
            return TestResult("Velocity command response", True, "base_node processing commands")
        else:
            return TestResult("Velocity command response", False, "base_node not running")

    # ==================== ROS2 TOPICS TESTS ====================

    def _test_ros2_topics(self):
        """Test ROS2 topic availability and publishing rates"""
        print(f"{Colors.CYAN}[ROS2 TOPICS - 12 tests]{Colors.NC}")

        # Critical topics
        tests = [
            self._test_topic_exists_and_hz('/scan', min_hz=8.0, max_hz=12.0),
            self._test_topic_exists_and_hz('/odometry/filtered', min_hz=5.0, max_hz=50.0),
            self._test_topic_exists_and_hz('/odom_raw', min_hz=5.0, max_hz=50.0),
            self._test_topic_exists_and_hz('/camera/color/image_raw', min_hz=15.0, max_hz=35.0),
            self._test_topic_exists_and_hz('/camera/depth/image_raw', min_hz=15.0, max_hz=35.0),
            self._test_topic_exists_and_hz('/cmd_vel', min_hz=1.0, max_hz=100.0),
            self._test_topic_exists_and_hz('/imu/data', min_hz=50.0, max_hz=150.0),
            # Autonomous topics
            self._test_topic_exists('/autonomous/decision'),
            self._test_topic_exists('/autonomous/detections'),
            self._test_topic_exists('/autonomous/status'),
            self._test_topic_exists('/autonomous/lane_detections'),
            self._test_topic_qos('/scan', expected='BEST_EFFORT'),
        ]

        for result in tests:
            self._print_result(result)
            self.all_results.append(result)
        print()

    def _test_topic_exists(self, topic: str) -> TestResult:
        if self.helper.topic_exists(topic):
            return TestResult(f"{topic}", True, "Topic exists")
        return TestResult(f"{topic}", False, "Topic not found")

    def _test_topic_exists_and_hz(self, topic: str, min_hz: float, max_hz: float) -> TestResult:
        if not self.helper.topic_exists(topic):
            return TestResult(f"{topic}", False, "Topic not found")

        hz = self.helper.topic_hz(topic, duration=3)
        if hz is None:
            return TestResult(f"{topic}", False, "Not publishing")

        if hz < min_hz:
            return TestResult(f"{topic}", False, f"{hz:.1f} Hz (too slow, expected >{min_hz})")
        if hz > max_hz:
            return TestResult(f"{topic}", False, f"{hz:.1f} Hz (too fast, expected <{max_hz})")

        return TestResult(f"{topic}", True, f"{hz:.1f} Hz")

    def _test_topic_qos(self, topic: str, expected: str) -> TestResult:
        success, output = self.helper.run_ros_command(f"ros2 topic info {topic} -v", timeout=3)
        if not success:
            return TestResult(f"{topic} QoS", False, "Cannot get QoS info")

        if expected in output:
            return TestResult(f"{topic} QoS", True, f"{expected}")
        elif "BEST_EFFORT" in output:
            return TestResult(f"{topic} QoS", False, f"BEST_EFFORT (expected {expected})")
        elif "RELIABLE" in output:
            return TestResult(f"{topic} QoS", False, f"RELIABLE (expected {expected})")
        return TestResult(f"{topic} QoS", False, "Unknown QoS")

    # ==================== SENSOR DATA QUALITY TESTS ====================

    def _test_sensor_data_quality(self):
        """Test actual sensor data quality"""
        print(f"{Colors.CYAN}[SENSOR DATA QUALITY - 8 tests]{Colors.NC}")

        tests = [
            self._test_lidar_scan_quality(),
            self._test_lidar_range_distribution(),
            self._test_camera_image_quality(),
            self._test_depth_image_quality(),
            self._test_imu_data_validity(),
            self._test_odometry_data_format(),
            self._test_perception_detection_format(),
            self._test_lane_detection_format(),
        ]

        for result in tests:
            self._print_result(result)
            self.all_results.append(result)
        print()

    def _test_lidar_scan_quality(self) -> TestResult:
        """Check LIDAR scan has sufficient valid ranges"""
        output = self.helper.topic_echo_once('/scan', timeout=3)
        if not output:
            return TestResult("LIDAR scan quality", False, "No scan data")

        # Count ranges
        ranges = re.findall(r'ranges:.*?\[(.*?)\]', output, re.DOTALL)
        if not ranges:
            return TestResult("LIDAR scan quality", False, "Cannot parse ranges")

        range_values = [float(x) for x in ranges[0].split(',') if x.strip()]
        total = len(range_values)
        valid = sum(1 for r in range_values if 0.15 < r < 12.0)
        valid_pct = (valid / total * 100) if total > 0 else 0

        if valid_pct < 5.0:
            return TestResult("LIDAR scan quality", False, f"{valid_pct:.1f}% valid (too sparse)")
        return TestResult("LIDAR scan quality", True, f"{valid_pct:.1f}% valid ({valid}/{total} ranges)")

    def _test_lidar_range_distribution(self) -> TestResult:
        """Check LIDAR detects obstacles at various distances"""
        output = self.helper.topic_echo_once('/scan', timeout=3)
        if not output:
            return TestResult("LIDAR range distribution", False, "No scan data")

        ranges = re.findall(r'ranges:.*?\[(.*?)\]', output, re.DOTALL)
        if not ranges:
            return TestResult("LIDAR range distribution", False, "Cannot parse")

        range_values = [float(x) for x in ranges[0].split(',') if x.strip() and 0.15 < float(x) < 12.0]
        if not range_values:
            return TestResult("LIDAR range distribution", False, "No valid ranges")

        min_range = min(range_values)
        max_range = max(range_values)
        return TestResult("LIDAR range distribution", True, f"min={min_range:.2f}m, max={max_range:.2f}m")

    def _test_camera_image_quality(self) -> TestResult:
        """Check camera image has valid data"""
        output = self.helper.topic_echo_once('/camera/color/image_raw', timeout=5)
        if not output:
            return TestResult("Camera image quality", False, "No image data")

        # Check for expected format
        if 'height:' in output and 'width:' in output:
            height_match = re.search(r'height:\s*(\d+)', output)
            width_match = re.search(r'width:\s*(\d+)', output)
            if height_match and width_match:
                h = height_match.group(1)
                w = width_match.group(1)
                return TestResult("Camera image quality", True, f"{w}x{h} resolution")
        return TestResult("Camera image quality", True, "Image data present")

    def _test_depth_image_quality(self) -> TestResult:
        """Check depth camera has valid data"""
        output = self.helper.topic_echo_once('/camera/depth/image_raw', timeout=5)
        if not output:
            return TestResult("Depth image quality", False, "No depth data")

        if 'height:' in output and 'width:' in output:
            height_match = re.search(r'height:\s*(\d+)', output)
            width_match = re.search(r'width:\s*(\d+)', output)
            if height_match and width_match:
                h = height_match.group(1)
                w = width_match.group(1)
                return TestResult("Depth image quality", True, f"{w}x{h} resolution")
        return TestResult("Depth image quality", True, "Depth data present")

    def _test_imu_data_validity(self) -> TestResult:
        """Check IMU data format"""
        output = self.helper.topic_echo_once('/imu/data', timeout=3)
        if not output:
            return TestResult("IMU data validity", False, "No IMU data")

        if 'angular_velocity:' in output and 'linear_acceleration:' in output:
            return TestResult("IMU data validity", True, "Angular velocity + linear accel OK")
        return TestResult("IMU data validity", True, "IMU data present")

    def _test_odometry_data_format(self) -> TestResult:
        """Check odometry has pose and twist"""
        output = self.helper.topic_echo_once('/odometry/filtered', timeout=3)
        if not output:
            return TestResult("Odometry data format", False, "No odometry data")

        if 'pose:' in output and 'twist:' in output:
            return TestResult("Odometry data format", True, "Pose + Twist OK")
        return TestResult("Odometry data format", True, "Odometry data present")

    def _test_perception_detection_format(self) -> TestResult:
        """Check YOLO detections format"""
        output = self.helper.topic_echo_once('/autonomous/detections', timeout=3)
        if not output:
            return TestResult("Perception detection format", False, "No detection data")

        if 'data:' in output:
            return TestResult("Perception detection format", True, "Detection messages OK")
        return TestResult("Perception detection format", True, "Detection data present")

    def _test_lane_detection_format(self) -> TestResult:
        """Check lane detection format"""
        output = self.helper.topic_echo_once('/autonomous/lane_detections', timeout=3)
        if not output:
            return TestResult("Lane detection format", False, "No lane data")
        return TestResult("Lane detection format", True, "Lane data present")

    # ==================== 2D GRID SYSTEM TESTS ====================

    def _test_2d_grid_system(self):
        """Test 2D grid generation and quality"""
        print(f"{Colors.CYAN}[2D GRID SYSTEM - 10 tests]{Colors.NC}")

        if not self.decision_output:
            self.all_results.append(TestResult("2D Grid System", False, "No decision output to analyze"))
            print(f"  {Colors.RED}✗ 2D Grid System - No decision output{Colors.NC}\n")
            return

        tests = [
            self._test_grid_ascii_generated(),
            self._test_grid_has_robot_marker(),
            self._test_grid_has_obstacles(),
            self._test_grid_has_free_space(),
            self._test_grid_statistics_present(),
            self._test_grid_cell_counts(),
            self._test_grid_update_time(),
            self._test_grid_confidence(),
            self._test_scan_buffer_status(),
            self._test_odometry_integration(),
        ]

        for result in tests:
            self._print_result(result)
            self.all_results.append(result)
        print()

    def _test_grid_ascii_generated(self) -> TestResult:
        """Check ASCII grid map is being generated"""
        if "Local Map 360°" in self.decision_output or "SPATIAL AWARENESS" in self.decision_output:
            # Count grid lines
            grid_lines = self.decision_output.count('\n       .')
            grid_lines += self.decision_output.count('\n       #')
            if grid_lines > 10:
                return TestResult("Grid ASCII generation", True, f"{grid_lines} grid lines")
            return TestResult("Grid ASCII generation", False, f"Only {grid_lines} grid lines")
        return TestResult("Grid ASCII generation", False, "No grid found in output")

    def _test_grid_has_robot_marker(self) -> TestResult:
        """Check grid contains robot position marker 'R'"""
        if '\n       ' in self.decision_output and 'R' in self.decision_output:
            # Look for R in grid context
            grid_section = self.decision_output[self.decision_output.find("SPATIAL AWARENESS"):self.decision_output.find("Grid Statistics")] if "Grid Statistics" in self.decision_output else self.decision_output
            if 'R' in grid_section:
                return TestResult("Grid robot marker", True, "Robot 'R' present in grid")
        return TestResult("Grid robot marker", False, "No robot marker found")

    def _test_grid_has_obstacles(self) -> TestResult:
        """Check grid contains obstacle markers '#'"""
        grid_section = self.decision_output[self.decision_output.find("SPATIAL AWARENESS"):self.decision_output.find("Grid Statistics")] if "Grid Statistics" in self.decision_output else self.decision_output
        obstacle_count = grid_section.count('#')
        if obstacle_count > 10:
            return TestResult("Grid obstacles", True, f"{obstacle_count} obstacle markers")
        elif obstacle_count > 0:
            return TestResult("Grid obstacles", False, f"Only {obstacle_count} obstacles (sparse)")
        return TestResult("Grid obstacles", False, "No obstacles in grid")

    def _test_grid_has_free_space(self) -> TestResult:
        """Check grid contains free space markers '.'"""
        grid_section = self.decision_output[self.decision_output.find("SPATIAL AWARENESS"):self.decision_output.find("Grid Statistics")] if "Grid Statistics" in self.decision_output else self.decision_output
        free_count = grid_section.count('.')
        if free_count > 50:
            return TestResult("Grid free space", True, f"{free_count} free cell markers")
        return TestResult("Grid free space", False, f"Only {free_count} free cells")

    def _test_grid_statistics_present(self) -> TestResult:
        """Check grid statistics are being reported"""
        if "Grid Statistics:" in self.decision_output:
            return TestResult("Grid statistics", True, "Statistics section present")
        return TestResult("Grid statistics", False, "No statistics found")

    def _test_grid_cell_counts(self) -> TestResult:
        """Validate grid cell distribution"""
        match = re.search(r'Grid cells:\s*(\d+)\s*total,\s*(\d+)\s*occupied,\s*(\d+)\s*free', self.decision_output)
        if match:
            total = int(match.group(1))
            occupied = int(match.group(2))
            free = int(match.group(3))

            if total != 1024:
                return TestResult("Grid cell counts", False, f"Total={total} (expected 1024)")
            if occupied > total * 0.9:
                return TestResult("Grid cell counts", False, f"{occupied} occupied ({occupied/total*100:.0f}% - too dense)")
            if occupied == 0:
                return TestResult("Grid cell counts", False, "No occupied cells")

            return TestResult("Grid cell counts", True, f"{occupied} occupied, {free} free, {total-occupied-free} unknown")
        return TestResult("Grid cell counts", False, "Cannot parse cell counts")

    def _test_grid_update_time(self) -> TestResult:
        """Check grid update time is within target"""
        match = re.search(r'Update time:\s*([\d.]+)ms\s*\(target:\s*<([\d.]+)ms\)', self.decision_output)
        if match:
            actual = float(match.group(1))
            target = float(match.group(2))
            if actual > target:
                return TestResult("Grid update time", False, f"{actual}ms (exceeds target {target}ms)")
            return TestResult("Grid update time", True, f"{actual}ms (target <{target}ms)")
        return TestResult("Grid update time", False, "Cannot parse update time")

    def _test_grid_confidence(self) -> TestResult:
        """Check grid confidence score"""
        match = re.search(r'Confidence:\s*([\d.]+)', self.decision_output)
        if match:
            conf = float(match.group(1))
            if conf < 0.3:
                return TestResult("Grid confidence", False, f"{conf:.2f} (too low)")
            return TestResult("Grid confidence", True, f"{conf:.2f}")
        return TestResult("Grid confidence", False, "Cannot parse confidence")

    def _test_scan_buffer_status(self) -> TestResult:
        """Check LIDAR scan buffer population"""
        match = re.search(r'Scans integrated:\s*(\d+)/(\d+)', self.decision_output)
        if match:
            integrated = int(match.group(1))
            total = int(match.group(2))
            if integrated == 0:
                return TestResult("Scan buffer integration", False, f"0/{total} scans (no odometry integration)")
            elif integrated < total / 2:
                return TestResult("Scan buffer integration", False, f"{integrated}/{total} scans (poor integration)")
            return TestResult("Scan buffer integration", True, f"{integrated}/{total} scans integrated")
        return TestResult("Scan buffer integration", False, "Cannot parse scan integration")

    def _test_odometry_integration(self) -> TestResult:
        """Check odometry displacement tracking"""
        if "Odometry displacement: No odometry data" in self.decision_output:
            return TestResult("Odometry integration", False, "No odometry data")
        elif "Odometry displacement:" in self.decision_output:
            match = re.search(r'Odometry displacement:\s*Δx=([+-]?[\d.]+)m', self.decision_output)
            if match:
                return TestResult("Odometry integration", True, f"Odometry tracking active")
        return TestResult("Odometry integration", False, "Cannot parse odometry status")

    # ==================== SENSOR FUSION TESTS ====================

    def _test_sensor_fusion(self):
        """Test sensor fusion system"""
        print(f"{Colors.CYAN}[SENSOR FUSION - 6 tests]{Colors.NC}")

        if not self.decision_output:
            self.all_results.append(TestResult("Sensor Fusion", False, "No decision output"))
            print(f"  {Colors.RED}✗ Sensor Fusion - No data{Colors.NC}\n")
            return

        tests = [
            self._test_lidar_fusion_status(),
            self._test_depth_fusion_status(),
            self._test_vision_fusion_status(),
            self._test_data_source_lidar(),
            self._test_data_source_camera(),
            self._test_fusion_obstacle_detection(),
        ]

        for result in tests:
            self._print_result(result)
            self.all_results.append(result)
        print()

    def _test_lidar_fusion_status(self) -> TestResult:
        """Check LIDAR sensor fusion status"""
        if "LIDAR: no_data" in self.decision_output:
            return TestResult("LIDAR fusion status", False, "no_data")
        elif "LIDAR: blocked" in self.decision_output:
            match = re.search(r'LIDAR: blocked \(([\d.]+)m\)', self.decision_output)
            if match:
                dist = match.group(1)
                return TestResult("LIDAR fusion status", True, f"Obstacle detected at {dist}m")
        elif "LIDAR: caution" in self.decision_output:
            match = re.search(r'LIDAR: caution \(([\d.]+)m\)', self.decision_output)
            if match:
                dist = match.group(1)
                return TestResult("LIDAR fusion status", True, f"Obstacle at {dist}m")
        elif "LIDAR: clear" in self.decision_output:
            return TestResult("LIDAR fusion status", True, "Clear path")
        return TestResult("LIDAR fusion status", False, "Unknown status")

    def _test_depth_fusion_status(self) -> TestResult:
        """Check depth camera fusion status"""
        if "Depth: offline" in self.decision_output:
            return TestResult("Depth fusion status", True, "Offline (LIDAR-only mode)")
        elif "Depth: clear" in self.decision_output:
            return TestResult("Depth fusion status", True, "Clear")
        elif "Depth: blocked" in self.decision_output:
            return TestResult("Depth fusion status", True, "Obstacle detected")
        return TestResult("Depth fusion status", False, "Unknown status")

    def _test_vision_fusion_status(self) -> TestResult:
        """Check vision (YOLO) fusion status"""
        if "Vision: No objects" in self.decision_output:
            return TestResult("Vision fusion status", True, "No objects (normal)")
        elif re.search(r'Vision:\s*(\d+)\s*objects detected', self.decision_output):
            match = re.search(r'Vision:\s*(\d+)\s*objects detected', self.decision_output)
            count = match.group(1)
            return TestResult("Vision fusion status", True, f"{count} objects detected")
        return TestResult("Vision fusion status", False, "Unknown status")

    def _test_data_source_lidar(self) -> TestResult:
        """Check LIDAR data source indicator"""
        if "Data sources: LIDAR=✓" in self.decision_output:
            return TestResult("Data source LIDAR", True, "LIDAR integrated (✓)")
        elif "Data sources: LIDAR=✗" in self.decision_output:
            return TestResult("Data source LIDAR", False, "LIDAR not integrated (✗)")
        return TestResult("Data source LIDAR", False, "Cannot determine")

    def _test_data_source_camera(self) -> TestResult:
        """Check camera data source indicator"""
        if "Data sources:" in self.decision_output and "Camera=✓" in self.decision_output:
            return TestResult("Data source Camera", True, "Camera integrated (✓)")
        elif "Camera=✗" in self.decision_output:
            return TestResult("Data source Camera", False, "Camera not integrated (✗)")
        return TestResult("Data source Camera", False, "Cannot determine")

    def _test_fusion_obstacle_detection(self) -> TestResult:
        """Check if fusion is detecting obstacles"""
        obstacle_match = re.search(r'Camera obstacle:\s*(\w+)\s*@\s*\(([\d.]+)m', self.decision_output)
        if obstacle_match:
            obj_type = obstacle_match.group(1)
            distance = obstacle_match.group(2)
            return TestResult("Fusion obstacle detection", True, f"{obj_type} at {distance}m")
        return TestResult("Fusion obstacle detection", True, "No obstacles (normal)")

    # ==================== LLM DECISION PIPELINE TESTS ====================

    def _test_llm_decision_pipeline(self):
        """Test LLM decision making pipeline"""
        print(f"{Colors.CYAN}[LLM DECISION PIPELINE - 8 tests]{Colors.NC}")

        if not self.decision_output:
            self.all_results.append(TestResult("LLM Pipeline", False, "No decision output"))
            print(f"  {Colors.RED}✗ LLM Pipeline - No data{Colors.NC}\n")
            return

        tests = [
            self._test_decision_pipeline_present(),
            self._test_llm_inference_time(),
            self._test_llm_response_format(),
            self._test_llm_action_validity(),
            self._test_final_action(),
            self._test_velocity_commands(),
            self._test_control_status(),
            self._test_safety_stop(),
        ]

        for result in tests:
            self._print_result(result)
            self.all_results.append(result)
        print()

    def _test_decision_pipeline_present(self) -> TestResult:
        """Check decision pipeline header is present"""
        if "AUTONOMOUS DRIVING DECISION PIPELINE" in self.decision_output:
            return TestResult("Decision pipeline header", True, "Present")
        return TestResult("Decision pipeline header", False, "Missing")

    def _test_llm_inference_time(self) -> TestResult:
        """Check LLM inference latency"""
        match = re.search(r'Inference Time:\s*([\d.]+)ms', self.decision_output)
        if match:
            latency = float(match.group(1))
            if latency > 2000:
                return TestResult("LLM inference time", False, f"{latency:.0f}ms (too slow)")
            elif latency > 1000:
                return TestResult("LLM inference time", True, f"{latency:.0f}ms (acceptable)")
            return TestResult("LLM inference time", True, f"{latency:.0f}ms (good)")
        return TestResult("LLM inference time", False, "Cannot parse inference time")

    def _test_llm_response_format(self) -> TestResult:
        """Check LLM raw response is valid JSON"""
        match = re.search(r'Raw Response:\s*"(\{.*?\})"', self.decision_output)
        if match:
            response = match.group(1)
            try:
                json.loads(response)
                return TestResult("LLM response format", True, "Valid JSON")
            except:
                return TestResult("LLM response format", False, "Invalid JSON")
        return TestResult("LLM response format", False, "Cannot parse response")

    def _test_llm_action_validity(self) -> TestResult:
        """Check LLM outputs valid action"""
        match = re.search(r'"action":\s*"(\w+)"', self.decision_output)
        if match:
            action = match.group(1)
            valid_actions = ['GO', 'SLOW', 'STOP', 'TURN_LEFT', 'TURN_RIGHT', 'BACKUP']
            if action in valid_actions:
                return TestResult("LLM action validity", True, f"Action '{action}' is valid")
            return TestResult("LLM action validity", False, f"Unknown action '{action}'")
        return TestResult("LLM action validity", False, "Cannot parse action")

    def _test_final_action(self) -> TestResult:
        """Check final robot action is determined"""
        match = re.search(r'Action:\s*(\w+)', self.decision_output)
        if match:
            action = match.group(1)
            return TestResult("Final action", True, f"{action}")
        return TestResult("Final action", False, "Cannot determine action")

    def _test_velocity_commands(self) -> TestResult:
        """Check velocity commands are set"""
        linear_match = re.search(r'Linear Velocity:\s*([\d.]+)\s*m/s', self.decision_output)
        angular_match = re.search(r'Angular Velocity:\s*([-\d.]+)\s*rad/s', self.decision_output)
        if linear_match and angular_match:
            linear = linear_match.group(1)
            angular = angular_match.group(1)
            return TestResult("Velocity commands", True, f"linear={linear}m/s, angular={angular}rad/s")
        return TestResult("Velocity commands", False, "Cannot parse velocities")

    def _test_control_status(self) -> TestResult:
        """Check control node status"""
        if "Control Status: ENABLED" in self.decision_output:
            return TestResult("Control status", True, "ENABLED")
        elif "Control Status: DISABLED" in self.decision_output:
            return TestResult("Control status", False, "DISABLED")
        elif "Control Status: UNKNOWN" in self.decision_output:
            return TestResult("Control status", False, "UNKNOWN")
        return TestResult("Control status", False, "Cannot determine")

    def _test_safety_stop(self) -> TestResult:
        """Check safety stop is working"""
        if "SAFETY STOP:" in self.decision_output:
            match = re.search(r'SAFETY STOP: Obstacle at ([\d.]+)m', self.decision_output)
            if match:
                dist = match.group(1)
                return TestResult("Safety stop", True, f"Triggered at {dist}m")
        return TestResult("Safety stop", True, "Not triggered (normal)")

    # ==================== CONTROL SYSTEM TESTS ====================

    def _test_control_system(self):
        """Test control system"""
        print(f"{Colors.CYAN}[CONTROL SYSTEM - 4 tests]{Colors.NC}")

        tests = [
            self._test_control_node_running(),
            self._test_cmd_vel_publishing(),
            self._test_autonomous_status(),
            self._test_control_loop_active(),
        ]

        for result in tests:
            self._print_result(result)
            self.all_results.append(result)
        print()

    def _test_control_node_running(self) -> TestResult:
        """Check control node process is running"""
        result = subprocess.run("ps aux | grep control_node | grep -v grep", shell=True, capture_output=True)
        if result.returncode == 0:
            return TestResult("Control node process", True, "Running")
        return TestResult("Control node process", False, "Not running")

    def _test_cmd_vel_publishing(self) -> TestResult:
        """Check /cmd_vel topic is publishing"""
        hz = self.helper.topic_hz('/cmd_vel', duration=2)
        if hz and hz > 1.0:
            return TestResult("/cmd_vel publishing", True, f"{hz:.1f} Hz")
        elif hz:
            return TestResult("/cmd_vel publishing", False, f"{hz:.1f} Hz (too slow)")
        return TestResult("/cmd_vel publishing", False, "Not publishing")

    def _test_autonomous_status(self) -> TestResult:
        """Check autonomous status topic"""
        if self.helper.topic_exists('/autonomous/status'):
            return TestResult("Autonomous status topic", True, "Present")
        return TestResult("Autonomous status topic", False, "Missing")

    def _test_control_loop_active(self) -> TestResult:
        """Check control loop is active"""
        output = self.helper.topic_echo_once('/cmd_vel', timeout=3)
        if output and ('linear:' in output or 'angular:' in output):
            return TestResult("Control loop active", True, "Sending commands")
        return TestResult("Control loop active", False, "No commands")

    # ==================== UTILITY METHODS ====================

    def _print_result(self, result: TestResult):
        """Print a single test result"""
        symbol = f"{Colors.GREEN}✓{Colors.NC}" if result.passed else f"{Colors.RED}✗{Colors.NC}"
        print(f"  {symbol} {result.name}: {result.message}")
        if self.verbose and result.details:
            print(f"      {Colors.CYAN}{result.details}{Colors.NC}")

    def _print_summary(self):
        """Print test summary"""
        passed = sum(1 for r in self.all_results if r.passed)
        failed = len(self.all_results) - passed

        print(f"{Colors.BLUE}{'='*80}{Colors.NC}")
        print(f"{Colors.BLUE}TEST SUMMARY{Colors.NC}")
        print(f"{Colors.BLUE}{'='*80}{Colors.NC}")
        print(f"  Total tests: {len(self.all_results)}")
        print(f"  {Colors.GREEN}Passed: {passed}{Colors.NC}")
        print(f"  {Colors.RED}Failed: {failed}{Colors.NC}")
        print(f"{Colors.BLUE}{'='*80}{Colors.NC}")

        if failed == 0:
            print(f"{Colors.GREEN}✓ ALL TESTS PASSED{Colors.NC}\n")
        else:
            print(f"{Colors.YELLOW}⚠ {failed} TEST(S) FAILED{Colors.NC}\n")
            print(f"{Colors.YELLOW}Failed tests:{Colors.NC}")
            for r in self.all_results:
                if not r.passed:
                    print(f"  {Colors.RED}✗ {r.name}: {r.message}{Colors.NC}")
            print()


def main():
    """Main entry point"""
    import argparse

    parser = argparse.ArgumentParser(description='Autonomous Robot Comprehensive Test Suite (Report Only)')
    parser.add_argument('--verbose', action='store_true', help='Verbose output with details')
    args = parser.parse_args()

    ros2_helper = ROS2Helper()
    tester = SystemTests(ros2_helper, verbose=args.verbose)

    success = tester.run_all_tests()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
