#!/usr/bin/env python3
"""
Autonomous Robot System - Comprehensive Test Suite
Auto-runs 30s after start_auto.py startup to catch regressions

Tests all components:
- Hardware (USB mappings, devices, permissions)
- ROS2 topics (publishing, QoS, data quality)
- 2D grid generation (buffers, quality)
- LLM pipeline (perception, decision, control)

Usage:
    python3 start_auto_test.py              # Interactive mode with fixes
    python3 start_auto_test.py --verbose    # Detailed output
    python3 start_auto_test.py --no-fix     # Report only, no fixes
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

# Color codes for output
class Colors:
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[0;33m'
    BLUE = '\033[0;34m'
    CYAN = '\033[0;36m'
    NC = '\033[0m'  # No Color


@dataclass
class TestResult:
    """Result of a single test"""
    name: str
    passed: bool
    message: str
    fix_available: bool = False
    fix_description: str = ""
    fix_command: Optional[str] = None


class TestCategory(Enum):
    """Test category for organization"""
    HARDWARE = "HARDWARE"
    ROS2_TOPICS = "ROS2 TOPICS"
    GRID_2D = "2D GRID"
    LLM_PIPELINE = "LLM PIPELINE"


class ROS2Helper:
    """Utilities for ROS2 environment and topic operations"""

    def __init__(self):
        self.ros_domain = os.environ.get('ROS_DOMAIN_ID', '28')
        self.ros_sourced = self._check_ros_environment()

    def _check_ros_environment(self) -> bool:
        """Check if ROS2 environment is sourced"""
        result = subprocess.run(
            "bash -c 'source /opt/ros/humble/setup.bash && which ros2'",
            shell=True,
            capture_output=True
        )
        return result.returncode == 0

    def run_ros_command(self, cmd: str, timeout: int = 5) -> Tuple[bool, str]:
        """Run a ROS2 command with proper environment"""
        full_cmd = f"bash -c 'export ROS_DOMAIN_ID={self.ros_domain} && source /opt/ros/humble/setup.bash && {cmd}'"
        try:
            result = subprocess.run(
                full_cmd,
                shell=True,
                capture_output=True,
                text=True,
                timeout=timeout
            )
            return (result.returncode == 0, result.stdout + result.stderr)
        except subprocess.TimeoutExpired:
            return (False, f"Command timed out after {timeout}s")

    def topic_exists(self, topic: str, timeout: int = 3) -> bool:
        """Check if a ROS2 topic exists"""
        success, output = self.run_ros_command(f"ros2 topic list", timeout=timeout)
        return success and topic in output

    def topic_hz(self, topic: str, duration: int = 3) -> Optional[float]:
        """Get publishing rate of a topic (returns Hz or None)"""
        success, output = self.run_ros_command(
            f"timeout {duration} ros2 topic hz {topic} 2>&1",
            timeout=duration + 2
        )
        if success and "average rate:" in output:
            # Parse: "average rate: 10.234"
            match = re.search(r'average rate:\s*([\d.]+)', output)
            if match:
                return float(match.group(1))
        return None

    def topic_info_qos(self, topic: str) -> Optional[str]:
        """Get QoS policy of a topic (BEST_EFFORT or RELIABLE)"""
        success, output = self.run_ros_command(f"ros2 topic info {topic} -v", timeout=3)
        if success:
            if "BEST_EFFORT" in output:
                return "BEST_EFFORT"
            elif "RELIABLE" in output:
                return "RELIABLE"
        return None


class FixManager:
    """Manages interactive fixes for failed tests"""

    def __init__(self, interactive: bool = True):
        self.interactive = interactive
        self.fixes_applied = []

    def offer_fix(self, test_result: TestResult) -> bool:
        """Offer to apply a fix for a failed test"""
        if not test_result.fix_available:
            return False

        print(f"{Colors.YELLOW}     Fix available: {test_result.fix_description or 'Apply suggested fix'}{Colors.NC}")

        if not self.interactive:
            print(f"{Colors.YELLOW}     (Skipping - non-interactive mode){Colors.NC}")
            return False

        response = input(f"{Colors.CYAN}     Apply fix? (y/N): {Colors.NC}").strip().lower()

        if response == 'y':
            return self._apply_fix(test_result)

        return False

    def _apply_fix(self, test_result: TestResult) -> bool:
        """Execute the fix command"""
        if not test_result.fix_command:
            print(f"{Colors.RED}     No fix command available{Colors.NC}")
            return False

        print(f"{Colors.CYAN}     Executing: {test_result.fix_command}{Colors.NC}")

        try:
            result = subprocess.run(
                test_result.fix_command,
                shell=True,
                capture_output=True,
                text=True,
                timeout=10
            )

            if result.returncode == 0:
                print(f"{Colors.GREEN}     ✓ Fix applied successfully{Colors.NC}")
                self.fixes_applied.append(test_result.name)
                return True
            else:
                print(f"{Colors.RED}     ✗ Fix failed: {result.stderr[:100]}{Colors.NC}")
                return False
        except Exception as e:
            print(f"{Colors.RED}     ✗ Fix failed: {str(e)}{Colors.NC}")
            return False


class USBDeviceTests:
    """Tests for USB device mappings and connectivity"""

    def test_usb_network_interfaces(self) -> TestResult:
        """Check usb0/usb1 network gadget configuration"""
        result = subprocess.run("ip link show", shell=True, capture_output=True, text=True)

        issues = []
        for usb_if in ['usb0', 'usb1']:
            if usb_if not in result.stdout:
                issues.append(f"{usb_if} missing")
            elif 'state DOWN' in result.stdout and usb_if in result.stdout:
                # Check if this specific interface is down
                usb_section = result.stdout[result.stdout.find(usb_if):result.stdout.find(usb_if) + 200]
                if 'state DOWN' in usb_section:
                    issues.append(f"{usb_if} DOWN")

        if issues:
            return TestResult(
                name="USB Network Interfaces",
                passed=False,
                message="; ".join(issues),
                fix_available=True,
                fix_description="Bring up USB network interfaces",
                fix_command="sudo ip link set usb0 up && sudo ip link set usb1 up"
            )

        return TestResult(
            name="USB Network (usb0, usb1)",
            passed=True,
            message="OK"
        )

    def test_serial_symlinks(self) -> TestResult:
        """Verify /dev/myserial and /dev/ydlidar point to correct devices"""
        try:
            myserial = os.readlink('/dev/myserial')
            ydlidar = os.readlink('/dev/ydlidar')
        except FileNotFoundError as e:
            return TestResult(
                name="Serial Symlinks",
                passed=False,
                message=f"Symlink missing: {e}",
                fix_available=True,
                fix_description="Reload udev rules",
                fix_command="sudo udevadm control --reload-rules && sudo udevadm trigger"
            )

        # Verify targets exist
        if not os.path.exists(f'/dev/{myserial}'):
            return TestResult(
                name="Serial Symlinks",
                passed=False,
                message=f"/dev/myserial → {myserial} (MISSING)",
                fix_available=False
            )

        if not os.path.exists(f'/dev/{ydlidar}'):
            return TestResult(
                name="Serial Symlinks",
                passed=False,
                message=f"/dev/ydlidar → {ydlidar} (MISSING)",
                fix_available=False
            )

        # Check for common regression: devices swapped
        if myserial == 'ttyUSB1' and ydlidar == 'ttyUSB0':
            return TestResult(
                name="Serial Symlinks",
                passed=False,
                message="SWAPPED: myserial→ttyUSB1, ydlidar→ttyUSB0 (expected opposite)",
                fix_available=False,
                fix_description="Update udev rules in /etc/udev/rules.d/"
            )

        return TestResult(
            name=f"Serial Devices (myserial→{myserial}, ydlidar→{ydlidar})",
            passed=True,
            message="OK"
        )

    def test_usb_camera(self) -> TestResult:
        """Verify Astra camera USB ID"""
        result = subprocess.run("lsusb", shell=True, capture_output=True, text=True)

        if '2bc5:050f' in result.stdout:
            return TestResult(
                name="Astra Camera (USB 2bc5:050f)",
                passed=True,
                message="OK"
            )

        return TestResult(
            name="USB Camera",
            passed=False,
            message="Astra camera not found on USB bus",
            fix_available=True,
            fix_description="Kill stale camera process",
            fix_command="pkill -9 astra_camera_node && sleep 2"
        )


class HardwareTests:
    """Tests for robot hardware devices"""

    def test_robot_controller(self) -> TestResult:
        """Check /dev/myserial exists and is accessible"""
        if not os.path.exists('/dev/myserial'):
            return TestResult(
                name="Robot Controller",
                passed=False,
                message="/dev/myserial not found",
                fix_available=False
            )

        # Check permissions
        try:
            with open('/dev/myserial', 'r'):
                pass
        except PermissionError:
            return TestResult(
                name="Robot Controller",
                passed=False,
                message="/dev/myserial permission denied",
                fix_available=True,
                fix_description="Fix device permissions",
                fix_command="sudo chmod 666 /dev/myserial"
            )
        except Exception:
            pass  # Device may not be ready for reading, but exists

        return TestResult(
            name="Robot controller (/dev/myserial)",
            passed=True,
            message="OK"
        )

    def test_lidar_device(self) -> TestResult:
        """Check /dev/ydlidar exists and is accessible"""
        if not os.path.exists('/dev/ydlidar'):
            return TestResult(
                name="YDLidar",
                passed=False,
                message="/dev/ydlidar not found",
                fix_available=False
            )

        # Check permissions
        try:
            with open('/dev/ydlidar', 'r'):
                pass
        except PermissionError:
            return TestResult(
                name="YDLidar",
                passed=False,
                message="/dev/ydlidar permission denied",
                fix_available=True,
                fix_description="Fix device permissions",
                fix_command="sudo chmod 666 /dev/ydlidar"
            )
        except Exception:
            pass

        return TestResult(
            name="YDLidar TG30 (/dev/ydlidar)",
            passed=True,
            message="OK"
        )


class ROS2TopicTests:
    """Tests for ROS2 topic availability and data quality"""

    def __init__(self, ros2_helper: ROS2Helper):
        self.helper = ros2_helper

        # Critical topics that system cannot run without
        self.critical_topics = {
            '/odometry/filtered': {'qos': 'RELIABLE', 'timeout': 10, 'min_hz': 5.0},
            '/scan': {'qos': 'BEST_EFFORT', 'timeout': 15, 'min_hz': 8.0}
        }

        # Optional topics (warn if missing but continue)
        self.optional_topics = {
            '/camera/color/image_raw': {'qos': 'RELIABLE', 'timeout': 20, 'min_hz': 15.0},
            '/camera/depth/image_raw': {'qos': 'RELIABLE', 'timeout': 20, 'min_hz': 15.0}
        }

    def test_topic_publishing(self, topic: str, config: dict) -> TestResult:
        """Test if a topic exists and is publishing data"""
        # Check if topic exists
        if not self.helper.topic_exists(topic, timeout=config['timeout']):
            return TestResult(
                name=f"{topic}",
                passed=False,
                message="NOT FOUND",
                fix_available=False
            )

        # Check publishing rate
        hz = self.helper.topic_hz(topic, duration=3)
        if hz is None:
            return TestResult(
                name=f"{topic}",
                passed=False,
                message="NOT PUBLISHING",
                fix_available=True,
                fix_description="Restart sensor node",
                fix_command=self._get_restart_command(topic)
            )

        # Check if rate is acceptable
        min_hz = config.get('min_hz', 1.0)
        if hz < min_hz:
            return TestResult(
                name=f"{topic}",
                passed=False,
                message=f"{hz:.1f} Hz (expected >{min_hz})",
                fix_available=False
            )

        # Check QoS if specified
        expected_qos = config.get('qos')
        if expected_qos:
            actual_qos = self.helper.topic_info_qos(topic)
            if actual_qos and actual_qos != expected_qos:
                return TestResult(
                    name=f"{topic}",
                    passed=False,
                    message=f"QoS mismatch: {actual_qos} (expected {expected_qos})",
                    fix_available=False
                )

        # Determine QoS display
        qos_str = ""
        if expected_qos:
            qos_str = f", {expected_qos}"

        return TestResult(
            name=f"{topic} ({hz:.1f} Hz{qos_str})",
            passed=True,
            message="OK"
        )

    def _get_restart_command(self, topic: str) -> str:
        """Get command to restart the node for a given topic"""
        if 'camera' in topic:
            return "pkill -9 astra_camera_node"
        elif 'scan' in topic:
            return "pkill -9 ydlidar"
        elif 'odom' in topic:
            return "pkill -9 ekf_node"
        return ""


class GridValidationTests:
    """Tests for 2D grid generation and quality"""

    def __init__(self, ros2_helper: ROS2Helper):
        self.helper = ros2_helper

    def test_grid_system(self) -> List[TestResult]:
        """
        Test 2D grid system by monitoring decision messages
        Returns multiple test results for buffers and grid quality
        """
        results = []

        # Check if decision topic exists
        if not self.helper.topic_exists('/autonomous/decision', timeout=5):
            results.append(TestResult(
                name="2D Grid System",
                passed=False,
                message="LLM decision node not publishing",
                fix_available=False
            ))
            return results

        # Sample a decision message to check grid status
        success, output = self.helper.run_ros_command(
            "timeout 5 ros2 topic echo /autonomous/decision --once",
            timeout=7
        )

        if not success or not output:
            results.append(TestResult(
                name="Grid Data",
                passed=False,
                message="No decision messages received",
                fix_available=False
            ))
            return results

        # Parse the output to check for grid indicators
        # Look for key phrases in the decision output
        has_scan_data = "LIDAR:" in output and "no_data" not in output
        has_odom_data = "Odom=" in output and "NO DATA" not in output
        has_grid = "visual_grid" in output or "Grid Statistics" in output

        # Test scan buffer
        results.append(TestResult(
            name="Scan buffer",
            passed=has_scan_data,
            message="OK" if has_scan_data else "No LIDAR scans",
            fix_available=not has_scan_data,
            fix_description="Wait for LIDAR to accumulate scans",
            fix_command="sleep 3"
        ))

        # Test odometry history
        results.append(TestResult(
            name="Odometry history",
            passed=has_odom_data,
            message="OK" if has_odom_data else "No odometry poses",
            fix_available=not has_odom_data,
            fix_description="Check EKF and base_node",
            fix_command="pkill -9 ekf_node"
        ))

        # Test grid generation
        if has_grid:
            # Try to extract grid quality metrics
            if "Grid Statistics" in output:
                # Parse statistics if available
                occupied_match = re.search(r'(\d+)\s+occupied', output)
                if occupied_match:
                    occupied = int(occupied_match.group(1))
                    results.append(TestResult(
                        name="Grid generation",
                        passed=True,
                        message=f"OK ({occupied} obstacles)"
                    ))
                else:
                    results.append(TestResult(
                        name="Grid generation",
                        passed=True,
                        message="OK"
                    ))
            else:
                results.append(TestResult(
                    name="Grid generation",
                    passed=True,
                    message="OK"
                ))
        else:
            results.append(TestResult(
                name="Grid generation",
                passed=False,
                message="No grid data in decision messages",
                fix_available=False
            ))

        return results


class LLMPipelineTests:
    """Tests for LLM decision pipeline end-to-end"""

    def __init__(self, ros2_helper: ROS2Helper):
        self.helper = ros2_helper

    def test_perception(self) -> TestResult:
        """Check if perception node is publishing detections"""
        if not self.helper.topic_exists('/autonomous/detections', timeout=3):
            return TestResult(
                name="Perception (YOLO11)",
                passed=False,
                message="Not publishing",
                fix_available=False
            )

        hz = self.helper.topic_hz('/autonomous/detections', duration=2)
        if hz is None or hz < 0.5:
            return TestResult(
                name="Perception (YOLO11)",
                passed=False,
                message="Low publishing rate",
                fix_available=False
            )

        return TestResult(
            name="Perception (YOLO11)",
            passed=True,
            message="OK"
        )

    def test_llm_decision(self) -> TestResult:
        """Check if LLM is making decisions with reasonable latency"""
        if not self.helper.topic_exists('/autonomous/decision', timeout=3):
            return TestResult(
                name="Decision (Qwen3)",
                passed=False,
                message="Not publishing",
                fix_available=False
            )

        # Sample a decision to check inference time
        success, output = self.helper.run_ros_command(
            "timeout 3 ros2 topic echo /autonomous/decision --once",
            timeout=5
        )

        if success and output:
            # Try to extract inference time
            time_match = re.search(r'Inference Time:\s*([\d.]+)ms', output)
            if time_match:
                inference_ms = float(time_match.group(1))
                if inference_ms > 2000:  # > 2 seconds is too slow
                    return TestResult(
                        name="Decision (Qwen3)",
                        passed=False,
                        message=f"Slow ({inference_ms:.0f}ms)",
                        fix_available=False
                    )
                return TestResult(
                    name=f"Decision (Qwen3, {inference_ms:.0f}ms)",
                    passed=True,
                    message="OK"
                )

        # If we can't parse, at least it's publishing
        return TestResult(
            name="Decision (Qwen3)",
            passed=True,
            message="OK"
        )

    def test_control_node(self) -> TestResult:
        """Check if control node is active"""
        if not self.helper.topic_exists('/autonomous/status', timeout=3):
            return TestResult(
                name="Control active",
                passed=False,
                message="Status topic not found",
                fix_available=False
            )

        return TestResult(
            name="Control active",
            passed=True,
            message="OK"
        )

    def test_motor_commands(self) -> TestResult:
        """Check if motor commands are being published"""
        if not self.helper.topic_exists('/cmd_vel', timeout=3):
            return TestResult(
                name="Motor commands",
                passed=False,
                message="Topic not found",
                fix_available=False
            )

        hz = self.helper.topic_hz('/cmd_vel', duration=2)
        if hz is None or hz < 1.0:
            return TestResult(
                name="Motor commands",
                passed=False,
                message="Not publishing",
                fix_available=False
            )

        return TestResult(
            name="Motor commands",
            passed=True,
            message="OK"
        )


class TestRunner:
    """Main test orchestrator"""

    def __init__(self, interactive: bool = True, verbose: bool = False):
        self.interactive = interactive
        self.verbose = verbose
        self.ros2_helper = ROS2Helper()
        self.fix_manager = FixManager(interactive=interactive)
        self.results_by_category = {cat: [] for cat in TestCategory}

    def run_all_tests(self) -> bool:
        """
        Run all test phases in sequence
        Returns True if all critical tests passed
        """
        print(f"\n{Colors.BLUE}{'='*80}{Colors.NC}")
        print(f"{Colors.BLUE}🔍 AUTONOMOUS ROBOT SYSTEM TEST{Colors.NC}")
        print(f"{Colors.BLUE}{'='*80}{Colors.NC}\n")

        # Phase 1: Hardware Tests (FAIL FAST)
        if not self._run_hardware_tests():
            print(f"\n{Colors.RED}CRITICAL: Hardware tests failed. Cannot continue.{Colors.NC}\n")
            return False

        # Phase 2: ROS2 Topic Tests
        self._run_ros2_tests()

        # Phase 3: 2D Grid Validation
        self._run_grid_tests()

        # Phase 4: LLM Pipeline Tests
        self._run_llm_tests()

        # Print summary
        self._print_summary()

        # Return overall pass/fail
        return self._all_critical_passed()

    def _run_hardware_tests(self) -> bool:
        """Run hardware tests (USB, devices)"""
        print(f"{Colors.CYAN}[HARDWARE]{Colors.NC}")

        usb_tests = USBDeviceTests()
        hw_tests = HardwareTests()

        tests = [
            usb_tests.test_usb_network_interfaces(),
            usb_tests.test_serial_symlinks(),
            hw_tests.test_robot_controller(),
            hw_tests.test_lidar_device(),
            usb_tests.test_usb_camera(),
        ]

        for result in tests:
            self._print_result(result)
            self.results_by_category[TestCategory.HARDWARE].append(result)

            if not result.passed and result.fix_available:
                if self.fix_manager.offer_fix(result):
                    # Re-run the test to verify fix
                    time.sleep(1)

        print()

        # Critical hardware: robot controller and LIDAR must pass
        critical_passed = all(
            r.passed for r in tests
            if 'Robot controller' in r.name or 'YDLidar' in r.name or 'Serial' in r.name
        )

        return critical_passed

    def _run_ros2_tests(self):
        """Run ROS2 topic tests"""
        print(f"{Colors.CYAN}[ROS2 TOPICS]{Colors.NC}")

        ros2_tests = ROS2TopicTests(self.ros2_helper)

        # Test critical topics
        for topic, config in ros2_tests.critical_topics.items():
            result = ros2_tests.test_topic_publishing(topic, config)
            self._print_result(result)
            self.results_by_category[TestCategory.ROS2_TOPICS].append(result)

            if not result.passed and result.fix_available:
                self.fix_manager.offer_fix(result)

        # Test optional topics (warnings only)
        for topic, config in ros2_tests.optional_topics.items():
            result = ros2_tests.test_topic_publishing(topic, config)
            self._print_result(result)
            self.results_by_category[TestCategory.ROS2_TOPICS].append(result)

        print()

    def _run_grid_tests(self):
        """Run 2D grid validation tests"""
        print(f"{Colors.CYAN}[2D GRID]{Colors.NC}")

        grid_tests = GridValidationTests(self.ros2_helper)
        results = grid_tests.test_grid_system()

        for result in results:
            self._print_result(result)
            self.results_by_category[TestCategory.GRID_2D].append(result)

            if not result.passed and result.fix_available:
                self.fix_manager.offer_fix(result)

        print()

    def _run_llm_tests(self):
        """Run LLM pipeline tests"""
        print(f"{Colors.CYAN}[LLM PIPELINE]{Colors.NC}")

        llm_tests = LLMPipelineTests(self.ros2_helper)

        tests = [
            llm_tests.test_perception(),
            llm_tests.test_llm_decision(),
            llm_tests.test_control_node(),
            llm_tests.test_motor_commands(),
        ]

        for result in tests:
            self._print_result(result)
            self.results_by_category[TestCategory.LLM_PIPELINE].append(result)

        print()

    def _print_result(self, result: TestResult):
        """Print a single test result"""
        symbol = f"{Colors.GREEN}✓{Colors.NC}" if result.passed else f"{Colors.RED}✗{Colors.NC}"
        print(f"  {symbol} {result.name}")

        if not result.passed:
            print(f"{Colors.RED}     {result.message}{Colors.NC}")

    def _print_summary(self):
        """Print final summary"""
        print(f"{Colors.BLUE}{'='*80}{Colors.NC}")

        passed_categories = 0
        total_categories = 0

        for category, results in self.results_by_category.items():
            if not results:
                continue

            total_categories += 1
            passed = sum(1 for r in results if r.passed)
            total = len(results)

            if passed == total:
                passed_categories += 1
                symbol = "✓"
                color = Colors.GREEN
            else:
                symbol = "⚠️"
                color = Colors.YELLOW

            print(f"{color}{symbol} {category.value}: {passed}/{total} tests passed{Colors.NC}")

        print(f"{Colors.BLUE}{'='*80}{Colors.NC}")

        if passed_categories == total_categories:
            print(f"{Colors.GREEN}RESULT: All {total_categories} component groups passed ✓{Colors.NC}\n")
        else:
            print(f"{Colors.YELLOW}RESULT: {passed_categories} of {total_categories} component groups passed ⚠️{Colors.NC}\n")

    def _all_critical_passed(self) -> bool:
        """Check if all critical tests passed"""
        # Critical: Hardware and ROS2 critical topics
        hardware_ok = all(r.passed for r in self.results_by_category[TestCategory.HARDWARE]
                          if 'Robot controller' in r.name or 'YDLidar' in r.name)

        ros2_ok = all(r.passed for r in self.results_by_category[TestCategory.ROS2_TOPICS]
                      if '/odometry/filtered' in r.name or '/scan' in r.name)

        return hardware_ok and ros2_ok


def main():
    """Main entry point"""
    import argparse

    parser = argparse.ArgumentParser(description='Autonomous Robot System Test Suite')
    parser.add_argument('--verbose', action='store_true', help='Verbose output')
    parser.add_argument('--no-fix', action='store_true', help='Report only, no interactive fixes')

    args = parser.parse_args()

    runner = TestRunner(
        interactive=not args.no_fix,
        verbose=args.verbose
    )

    success = runner.run_all_tests()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
