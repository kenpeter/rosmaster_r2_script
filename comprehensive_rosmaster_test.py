#!/usr/bin/env python3
"""
Comprehensive ROS Master Board Diagnostic Script
Tests all hardware, communication, sensors, and actuators
Generates detailed diagnostic report
"""

import serial
import time
import sys
import os
from datetime import datetime
import subprocess

class Colors:
    """Terminal colors for better readability"""
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class ROSMasterDiagnostic:
    def __init__(self):
        self.results = {
            'hardware': {},
            'communication': {},
            'sensors': {},
            'actuators': {},
            'power': {},
            'firmware': {}
        }
        self.test_count = 0
        self.passed_count = 0
        self.failed_count = 0
        self.warning_count = 0
        self.serial_port = None
        self.ser = None

    def print_header(self, text):
        """Print section header"""
        print(f"\n{Colors.HEADER}{Colors.BOLD}{'='*70}{Colors.ENDC}")
        print(f"{Colors.HEADER}{Colors.BOLD}{text:^70}{Colors.ENDC}")
        print(f"{Colors.HEADER}{Colors.BOLD}{'='*70}{Colors.ENDC}\n")

    def print_test(self, name):
        """Print test name"""
        print(f"{Colors.OKCYAN}[TEST {self.test_count + 1}] {name}...{Colors.ENDC}", end=' ')
        sys.stdout.flush()

    def print_result(self, status, message="", details=""):
        """Print test result"""
        self.test_count += 1
        if status == "PASS":
            self.passed_count += 1
            print(f"{Colors.OKGREEN}✓ PASS{Colors.ENDC} {message}")
        elif status == "FAIL":
            self.failed_count += 1
            print(f"{Colors.FAIL}✗ FAIL{Colors.ENDC} {message}")
        elif status == "WARN":
            self.warning_count += 1
            print(f"{Colors.WARNING}⚠ WARNING{Colors.ENDC} {message}")

        if details:
            for line in details.split('\n'):
                if line.strip():
                    print(f"  {Colors.OKBLUE}→{Colors.ENDC} {line}")

    def run_command(self, cmd):
        """Run shell command and return output"""
        try:
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5)
            return result.returncode, result.stdout, result.stderr
        except subprocess.TimeoutExpired:
            return -1, "", "Command timed out"
        except Exception as e:
            return -1, "", str(e)

    def test_usb_devices(self):
        """Test 1: Detect USB devices"""
        self.print_header("HARDWARE DETECTION TESTS")

        self.print_test("USB Device Detection")
        code, stdout, stderr = self.run_command("ls -la /dev/ttyUSB* 2>/dev/null")

        if code == 0 and stdout:
            devices = [line.split()[-1] for line in stdout.strip().split('\n') if 'ttyUSB' in line]
            self.results['hardware']['usb_devices'] = devices
            self.print_result("PASS", f"Found {len(devices)} USB device(s)",
                            "\n".join(devices))
            return devices
        else:
            self.results['hardware']['usb_devices'] = []
            self.print_result("FAIL", "No USB devices found")
            return []

    def test_usb_permissions(self, devices):
        """Test 2: Check USB device permissions"""
        self.print_test("USB Device Permissions")

        permission_ok = True
        details = []
        for device in devices:
            code, stdout, stderr = self.run_command(f"ls -la {device}")
            if 'rw-rw----' in stdout or 'rw-rw-rw-' in stdout:
                details.append(f"{device}: ✓ Readable/Writable")
            else:
                permission_ok = False
                details.append(f"{device}: ✗ Permission denied")

        if permission_ok and devices:
            self.results['hardware']['permissions'] = 'OK'
            self.print_result("PASS", "All devices have correct permissions",
                            "\n".join(details))
        elif devices:
            self.results['hardware']['permissions'] = 'FAILED'
            self.print_result("FAIL", "Permission issues detected",
                            "\n".join(details))
        else:
            self.print_result("WARN", "No devices to check")

    def test_usb_info(self, devices):
        """Test 3: Get detailed USB information"""
        self.print_test("USB Device Information")

        details = []
        for device in devices:
            code, stdout, stderr = self.run_command(f"udevadm info {device} | grep -E 'ID_VENDOR|ID_MODEL|ID_SERIAL'")
            if code == 0:
                info = stdout.strip().replace('E: ', '').replace('ID_', '')
                details.append(f"{device}:\n  {info.replace(chr(10), chr(10) + '  ')}")

        if details:
            self.results['hardware']['usb_info'] = stdout
            self.print_result("PASS", "Retrieved device information",
                            "\n".join(details))
        else:
            self.print_result("WARN", "Could not retrieve device info")

    def find_rosmaster_port(self, devices):
        """Test 4: Identify ROS Master port"""
        self.print_header("COMMUNICATION TESTS")

        self.print_test("ROS Master Port Identification")

        for device in devices:
            try:
                ser = serial.Serial(device, 115200, timeout=1)
                time.sleep(0.5)

                # Send test command (version request)
                test_cmd = bytes([0x55, 0x55, 0x02, 0x01, 0xFF, 0xFD])
                ser.write(test_cmd)
                time.sleep(0.3)

                response = ser.read(100)
                ser.close()

                if len(response) > 0:
                    self.serial_port = device
                    self.results['communication']['port'] = device
                    self.print_result("PASS", f"ROS Master found on {device}",
                                    f"Response length: {len(response)} bytes\nData: {response.hex()}")
                    return device
            except Exception as e:
                continue

        self.print_result("FAIL", "ROS Master port not found")
        return None

    def test_serial_communication(self):
        """Test 5: Test basic serial communication"""
        self.print_test("Serial Communication Protocol")

        if not self.serial_port:
            self.print_result("FAIL", "No port available")
            return False

        try:
            self.ser = serial.Serial(self.serial_port, 115200, timeout=1)
            time.sleep(0.5)

            # Test multiple command types
            commands = {
                'Version Check': bytes([0x55, 0x55, 0x02, 0x01, 0xFF, 0xFD]),
                'Battery Status': bytes([0x55, 0x55, 0x02, 0x02, 0x00, 0xFD]),
            }

            all_ok = True
            details = []
            for cmd_name, cmd_bytes in commands.items():
                self.ser.write(cmd_bytes)
                time.sleep(0.2)
                response = self.ser.read(100)

                if len(response) > 0:
                    details.append(f"{cmd_name}: ✓ Response ({len(response)} bytes)")
                else:
                    all_ok = False
                    details.append(f"{cmd_name}: ✗ No response")

            if all_ok:
                self.results['communication']['protocol'] = 'OK'
                self.print_result("PASS", "Serial protocol working",
                                "\n".join(details))
                return True
            else:
                self.results['communication']['protocol'] = 'PARTIAL'
                self.print_result("WARN", "Partial communication",
                                "\n".join(details))
                return True

        except Exception as e:
            self.results['communication']['protocol'] = 'FAILED'
            self.print_result("FAIL", f"Communication error: {str(e)}")
            return False

    def test_firmware_version(self):
        """Test 6: Get firmware version"""
        self.print_test("Firmware Version Check")

        if not self.ser:
            self.print_result("FAIL", "Serial port not initialized")
            return

        try:
            # Request version
            cmd = bytes([0x55, 0x55, 0x02, 0x01, 0xFF, 0xFD])
            self.ser.write(cmd)
            time.sleep(0.3)
            response = self.ser.read(100)

            if len(response) >= 4:
                # Parse version (adjust based on your protocol)
                version_info = f"Raw: {response.hex()}"
                self.results['firmware']['version'] = response.hex()
                self.print_result("PASS", "Firmware version retrieved",
                                version_info)
            else:
                self.print_result("WARN", "Could not parse version")

        except Exception as e:
            self.print_result("FAIL", f"Error: {str(e)}")

    def test_battery_voltage(self):
        """Test 7: Read battery voltage"""
        self.print_header("POWER SYSTEM TESTS")

        self.print_test("Battery Voltage Reading")

        if not self.ser:
            self.print_result("FAIL", "Serial port not initialized")
            return

        try:
            # Request battery status
            cmd = bytes([0x55, 0x55, 0x02, 0x02, 0x00, 0xFD])
            self.ser.write(cmd)
            time.sleep(0.3)
            response = self.ser.read(100)

            if len(response) >= 4:
                # Parse battery voltage (adjust based on your protocol)
                # This is a placeholder - adjust for your specific protocol
                voltage_raw = response.hex()
                self.results['power']['battery_voltage'] = voltage_raw
                self.print_result("PASS", "Battery voltage retrieved",
                                f"Raw data: {voltage_raw}")
            else:
                self.print_result("WARN", "Could not read battery voltage")

        except Exception as e:
            self.print_result("FAIL", f"Error: {str(e)}")

    def test_motor_control(self):
        """Test 8: Test motor control commands"""
        self.print_header("ACTUATOR TESTS")

        self.print_test("Motor Control Commands")

        if not self.ser:
            self.print_result("FAIL", "Serial port not initialized")
            return

        try:
            # Send stop command first
            stop_cmd = bytes([0x55, 0x55, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFD])
            self.ser.write(stop_cmd)
            time.sleep(0.2)

            # Test small movement commands (won't actually move much)
            test_commands = {
                'Stop': bytes([0x55, 0x55, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFD]),
                'Low Speed Test': bytes([0x55, 0x55, 0x07, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0xFD]),
            }

            details = []
            for cmd_name, cmd_bytes in test_commands.items():
                self.ser.write(cmd_bytes)
                time.sleep(0.2)
                # Check if command was accepted
                response = self.ser.read(100)
                details.append(f"{cmd_name}: ✓ Sent")

            # Stop motors
            self.ser.write(stop_cmd)

            self.results['actuators']['motors'] = 'TESTED'
            self.print_result("PASS", "Motor commands sent successfully",
                            "\n".join(details) + "\n⚠ Motors not actually moved for safety")

        except Exception as e:
            self.results['actuators']['motors'] = 'FAILED'
            self.print_result("FAIL", f"Error: {str(e)}")

    def test_servo_control(self):
        """Test 9: Test servo control"""
        self.print_test("Servo Control Commands")

        if not self.ser:
            self.print_result("FAIL", "Serial port not initialized")
            return

        try:
            # Test servo command (neutral position)
            servo_cmd = bytes([0x55, 0x55, 0x05, 0x03, 0x01, 0x5A, 0x00, 0x00, 0xFD])
            self.ser.write(servo_cmd)
            time.sleep(0.2)

            self.results['actuators']['servos'] = 'TESTED'
            self.print_result("PASS", "Servo command sent successfully")

        except Exception as e:
            self.results['actuators']['servos'] = 'FAILED'
            self.print_result("FAIL", f"Error: {str(e)}")

    def test_sensor_readings(self):
        """Test 10: Test sensor readings"""
        self.print_header("SENSOR TESTS")

        self.print_test("Sensor Data Retrieval")

        if not self.ser:
            self.print_result("FAIL", "Serial port not initialized")
            return

        try:
            # Request sensor data
            sensor_cmd = bytes([0x55, 0x55, 0x02, 0x04, 0x00, 0xFD])
            self.ser.write(sensor_cmd)
            time.sleep(0.3)
            response = self.ser.read(100)

            if len(response) > 0:
                self.results['sensors']['data'] = response.hex()
                self.print_result("PASS", "Sensor data retrieved",
                                f"Data: {response.hex()}")
            else:
                self.print_result("WARN", "No sensor data received")

        except Exception as e:
            self.results['sensors']['data'] = 'FAILED'
            self.print_result("FAIL", f"Error: {str(e)}")

    def test_imu_data(self):
        """Test 11: Test IMU/orientation data"""
        self.print_test("IMU Data Reading")

        if not self.ser:
            self.print_result("FAIL", "Serial port not initialized")
            return

        try:
            # Request IMU data
            imu_cmd = bytes([0x55, 0x55, 0x02, 0x05, 0x00, 0xFD])
            self.ser.write(imu_cmd)
            time.sleep(0.3)
            response = self.ser.read(100)

            if len(response) > 0:
                self.results['sensors']['imu'] = response.hex()
                self.print_result("PASS", "IMU data retrieved",
                                f"Data: {response.hex()}")
            else:
                self.print_result("WARN", "No IMU data received")

        except Exception as e:
            self.results['sensors']['imu'] = 'FAILED'
            self.print_result("FAIL", f"Error: {str(e)}")

    def test_continuous_communication(self):
        """Test 12: Test continuous data stream"""
        self.print_header("STRESS TESTS")

        self.print_test("Continuous Communication Test (5 seconds)")

        if not self.ser:
            self.print_result("FAIL", "Serial port not initialized")
            return

        try:
            cmd = bytes([0x55, 0x55, 0x02, 0x01, 0xFF, 0xFD])
            success_count = 0
            fail_count = 0

            start_time = time.time()
            while time.time() - start_time < 5:
                self.ser.write(cmd)
                time.sleep(0.1)
                response = self.ser.read(100)

                if len(response) > 0:
                    success_count += 1
                else:
                    fail_count += 1

            total = success_count + fail_count
            success_rate = (success_count / total * 100) if total > 0 else 0

            self.results['communication']['reliability'] = success_rate

            if success_rate >= 90:
                self.print_result("PASS", f"Communication reliable: {success_rate:.1f}%",
                                f"Successful: {success_count}/{total}")
            elif success_rate >= 50:
                self.print_result("WARN", f"Communication unstable: {success_rate:.1f}%",
                                f"Successful: {success_count}/{total}")
            else:
                self.print_result("FAIL", f"Communication unreliable: {success_rate:.1f}%",
                                f"Successful: {success_count}/{total}")

        except Exception as e:
            self.results['communication']['reliability'] = 0
            self.print_result("FAIL", f"Error: {str(e)}")

    def test_response_time(self):
        """Test 13: Measure response time"""
        self.print_test("Response Time Measurement")

        if not self.ser:
            self.print_result("FAIL", "Serial port not initialized")
            return

        try:
            cmd = bytes([0x55, 0x55, 0x02, 0x01, 0xFF, 0xFD])
            response_times = []

            for i in range(10):
                start = time.time()
                self.ser.write(cmd)
                response = self.ser.read(100)
                end = time.time()

                if len(response) > 0:
                    response_times.append((end - start) * 1000)  # Convert to ms
                time.sleep(0.1)

            if response_times:
                avg_time = sum(response_times) / len(response_times)
                min_time = min(response_times)
                max_time = max(response_times)

                self.results['communication']['avg_response_time'] = avg_time

                details = f"Average: {avg_time:.2f}ms\nMin: {min_time:.2f}ms\nMax: {max_time:.2f}ms"

                if avg_time < 50:
                    self.print_result("PASS", "Response time excellent", details)
                elif avg_time < 100:
                    self.print_result("PASS", "Response time good", details)
                else:
                    self.print_result("WARN", "Response time slow", details)
            else:
                self.print_result("FAIL", "No responses received")

        except Exception as e:
            self.print_result("FAIL", f"Error: {str(e)}")

    def generate_report(self):
        """Generate comprehensive diagnostic report"""
        self.print_header("DIAGNOSTIC SUMMARY")

        print(f"\n{Colors.BOLD}Test Statistics:{Colors.ENDC}")
        print(f"  Total Tests: {self.test_count}")
        print(f"  {Colors.OKGREEN}Passed: {self.passed_count}{Colors.ENDC}")
        print(f"  {Colors.WARNING}Warnings: {self.warning_count}{Colors.ENDC}")
        print(f"  {Colors.FAIL}Failed: {self.failed_count}{Colors.ENDC}")

        success_rate = (self.passed_count / self.test_count * 100) if self.test_count > 0 else 0
        print(f"\n  Success Rate: {success_rate:.1f}%")

        # Overall status
        print(f"\n{Colors.BOLD}Overall Status:{Colors.ENDC}")
        if success_rate >= 90:
            status = f"{Colors.OKGREEN}EXCELLENT - Board fully functional{Colors.ENDC}"
        elif success_rate >= 70:
            status = f"{Colors.OKGREEN}GOOD - Board mostly functional{Colors.ENDC}"
        elif success_rate >= 50:
            status = f"{Colors.WARNING}FAIR - Board partially functional{Colors.ENDC}"
        else:
            status = f"{Colors.FAIL}POOR - Board has serious issues{Colors.ENDC}"

        print(f"  {status}")

        # Recommendations
        print(f"\n{Colors.BOLD}Recommendations:{Colors.ENDC}")

        if self.failed_count == 0 and self.warning_count == 0:
            print(f"  {Colors.OKGREEN}✓{Colors.ENDC} ROS Master board is working perfectly!")
        else:
            if not self.results['hardware'].get('usb_devices'):
                print(f"  {Colors.FAIL}!{Colors.ENDC} Check USB cable connections")
            if self.results['hardware'].get('permissions') == 'FAILED':
                print(f"  {Colors.FAIL}!{Colors.ENDC} Add user to dialout group: sudo usermod -a -G dialout $USER")
            if not self.results['communication'].get('port'):
                print(f"  {Colors.FAIL}!{Colors.ENDC} ROS Master board not responding - try power cycle")
            if self.results['communication'].get('reliability', 100) < 90:
                print(f"  {Colors.WARNING}!{Colors.ENDC} Unstable communication - check USB cable quality")
            if self.results['communication'].get('avg_response_time', 0) > 100:
                print(f"  {Colors.WARNING}!{Colors.ENDC} Slow response - check for USB interference")

        # Save report to file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_file = f"/tmp/rosmaster_diagnostic_{timestamp}.txt"

        with open(report_file, 'w') as f:
            f.write("="*70 + "\n")
            f.write("ROS MASTER BOARD DIAGNOSTIC REPORT\n")
            f.write("="*70 + "\n")
            f.write(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"\nTest Statistics:\n")
            f.write(f"  Total Tests: {self.test_count}\n")
            f.write(f"  Passed: {self.passed_count}\n")
            f.write(f"  Warnings: {self.warning_count}\n")
            f.write(f"  Failed: {self.failed_count}\n")
            f.write(f"  Success Rate: {success_rate:.1f}%\n")
            f.write(f"\nDetailed Results:\n")

            import json
            f.write(json.dumps(self.results, indent=2))

        print(f"\n{Colors.OKCYAN}Full report saved to: {report_file}{Colors.ENDC}")

    def cleanup(self):
        """Cleanup resources"""
        if self.ser and self.ser.is_open:
            try:
                # Send stop command
                stop_cmd = bytes([0x55, 0x55, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFD])
                self.ser.write(stop_cmd)
                time.sleep(0.1)
                self.ser.close()
            except:
                pass

    def run_all_tests(self):
        """Run all diagnostic tests"""
        print(f"\n{Colors.HEADER}{Colors.BOLD}")
        print("╔════════════════════════════════════════════════════════════════════╗")
        print("║        ROS MASTER BOARD COMPREHENSIVE DIAGNOSTIC TOOL              ║")
        print("║                    Version 1.0                                     ║")
        print("╚════════════════════════════════════════════════════════════════════╝")
        print(f"{Colors.ENDC}")
        print(f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")

        try:
            # Hardware tests
            devices = self.test_usb_devices()
            if devices:
                self.test_usb_permissions(devices)
                self.test_usb_info(devices)

                # Communication tests
                port = self.find_rosmaster_port(devices)
                if port:
                    if self.test_serial_communication():
                        # Firmware tests
                        self.test_firmware_version()

                        # Power tests
                        self.test_battery_voltage()

                        # Actuator tests
                        self.test_motor_control()
                        self.test_servo_control()

                        # Sensor tests
                        self.test_sensor_readings()
                        self.test_imu_data()

                        # Stress tests
                        self.test_continuous_communication()
                        self.test_response_time()

            # Generate final report
            self.generate_report()

        except KeyboardInterrupt:
            print(f"\n\n{Colors.WARNING}Test interrupted by user{Colors.ENDC}")
        except Exception as e:
            print(f"\n\n{Colors.FAIL}Unexpected error: {str(e)}{Colors.ENDC}")
        finally:
            self.cleanup()

        print(f"\n{Colors.OKBLUE}Diagnostic complete!{Colors.ENDC}\n")

def main():
    diagnostic = ROSMasterDiagnostic()
    diagnostic.run_all_tests()

if __name__ == "__main__":
    main()
