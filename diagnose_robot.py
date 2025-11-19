#!/usr/bin/env python3
"""
Yahboom Rosmaster R2 - Comprehensive Diagnostic Tool
All-in-one diagnostic for hardware, sensors, and connectivity

Usage:
  ./diagnose_robot.py           # Run full diagnostic
  ./diagnose_robot.py --quick   # Quick health check
  ./diagnose_robot.py --fix     # Auto-fix common issues
"""

import subprocess
import sys
import time
import serial
import os
from pathlib import Path

# Color codes
class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    END = '\033[0m'

def print_header(text):
    print(f"\n{Colors.BLUE}{'='*70}{Colors.END}")
    print(f"{Colors.BOLD}{text:^70}{Colors.END}")
    print(f"{Colors.BLUE}{'='*70}{Colors.END}\n")

def print_test(name):
    print(f"{Colors.CYAN}[TEST]{Colors.END} {name}...", end=" ", flush=True)

def print_pass(msg="PASS"):
    print(f"{Colors.GREEN}✅ {msg}{Colors.END}")

def print_fail(msg="FAIL"):
    print(f"{Colors.RED}❌ {msg}{Colors.END}")

def print_warn(msg):
    print(f"{Colors.YELLOW}⚠️  {msg}{Colors.END}")

def run_cmd(cmd, timeout=5):
    """Run shell command and return output"""
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=timeout
        )
        return result.returncode == 0, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return False, "", "Timeout"

# ============================================================================
#  1. USB DEVICE DETECTION
# ============================================================================
def test_usb_devices():
    print_header("USB Device Detection")

    results = {}

    # Check ROSMaster board
    print_test("ROSMaster board")
    success, out, _ = run_cmd("lsusb | grep -i '1a86:7523'")
    if success:
        print_pass("Found")
        results['rosmaster'] = True
    else:
        print_fail("Not found")
        results['rosmaster'] = False

    # Check YDLiDAR
    print_test("YDLiDAR")
    success, out, _ = run_cmd("lsusb | grep -i '10c4:ea60\\|1a86:7523'")
    if success or os.path.exists('/dev/ydlidar'):
        print_pass("Found")
        results['lidar'] = True
    else:
        print_fail("Not found")
        results['lidar'] = False

    # Check Camera
    print_test("Astra Camera")
    success, out, _ = run_cmd("lsusb | grep -i '2bc5:060f\\|Orbbec'")
    if success:
        print_pass("Found")
        results['camera'] = True
    else:
        print_fail("Not found")
        results['camera'] = False

    return results

# ============================================================================
#  2. ROSMASTER BOARD TEST
# ============================================================================
def test_rosmaster():
    print_header("ROSMaster Board Communication")

    port_path = None
    for p in ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2']:
        if os.path.exists(p):
            # Check if it's the ROSMaster by looking for the distinctive data pattern
            try:
                with serial.Serial(p, 115200, timeout=1) as ser:
                    time.sleep(0.5)
                    data = ser.read(100)
                    if b'\xff\xfb' in data:  # ROSMaster signature
                        port_path = p
                        break
            except:
                continue

    if not port_path:
        print_fail("ROSMaster port not found")
        return False

    print_test(f"ROSMaster on {port_path}")
    print_pass()

    try:
        with serial.Serial(port_path, 115200, timeout=2) as ser:
            time.sleep(0.5)

            # Test sensor data read
            print_test("Reading sensor data")
            data = ser.read(200)
            if len(data) > 50:
                print_pass(f"{len(data)} bytes")
            else:
                print_warn(f"Only {len(data)} bytes")

            # Test motor command (safe - just sends command)
            print_test("Motor command interface")
            cmd = bytes([0xff, 0xfe, 0x0c, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xed])
            ser.write(cmd)
            time.sleep(0.1)
            print_pass("Command sent")

        return True

    except Exception as e:
        print_fail(str(e))
        return False

# ============================================================================
#  3. YDLIDAR TEST
# ============================================================================
def test_ydlidar():
    print_header("YDLiDAR Motor & Communication")

    # Find LiDAR port
    lidar_port = None
    for p in ['/dev/ydlidar', '/dev/ttyUSB1', '/dev/ttyUSB2', '/dev/ttyUSB0']:
        if os.path.exists(p):
            try:
                with serial.Serial(p, 512000, timeout=1) as ser:
                    ser.setDTR(False)
                    time.sleep(1)
                    data = ser.read(50)
                    # Check if NOT ROSMaster data
                    if not (b'\xff\xfb' in data and len(data) > 30):
                        # Send YDLIDAR command
                        ser.write(bytes([0xA5, 0x90]))
                        time.sleep(0.1)
                        response = ser.read(50)
                        if b'\xa5\x5a' in response or len(response) > 10:
                            lidar_port = p
                            break
            except:
                continue

    if not lidar_port:
        print_warn("YDLiDAR port auto-detection failed")
        lidar_port = '/dev/ydlidar'

    print_test(f"Testing {lidar_port}")

    try:
        with serial.Serial(lidar_port, 512000, timeout=2) as ser:
            # Test DTR motor control
            print_test("Motor control (DTR)")
            ser.setDTR(False)
            time.sleep(3)
            ser.reset_input_buffer()
            data = ser.read(100)

            if len(data) > 10:
                print_pass(f"Motor spinning ({len(data)} bytes)")
                motor_ok = True
            else:
                print_warn("Motor may not be spinning")
                motor_ok = False

            # Try scan command
            if motor_ok:
                print_test("Scan data stream")
                data = ser.read(200)
                if b'\xAA\x55' in data:
                    print_pass("A1 scan packets detected")
                elif len(data) > 50:
                    print_warn("Data flowing but no standard headers")
                else:
                    print_warn("No scan data")

            ser.setDTR(True)  # Motor off
            return motor_ok

    except Exception as e:
        print_fail(str(e))
        return False

# ============================================================================
#  4. ROS2 SYSTEM TEST
# ============================================================================
def test_ros2_topics():
    print_header("ROS2 System Check")

    # Check if ROS2 system is running
    print_test("ROS2 nodes active")
    success, out, _ = run_cmd("ros2 node list 2>/dev/null", timeout=3)
    if success and len(out.strip()) > 0:
        node_count = len(out.strip().split('\n'))
        print_pass(f"{node_count} nodes running")
    else:
        print_warn("No ROS2 nodes running")
        return False

    # Check key topics
    topics_to_check = [
        ('/cmd_vel', 'Motor control'),
        ('/imu/data', 'IMU sensor'),
        ('/camera/depth/image_raw', 'Depth camera'),
        ('/odom_raw', 'Odometry'),
    ]

    for topic, name in topics_to_check:
        print_test(f"{name} ({topic})")
        success, out, _ = run_cmd(f"timeout 1 ros2 topic info {topic} 2>/dev/null")
        if success:
            print_pass()
        else:
            print_fail()

    # Check for LiDAR scan topic
    print_test("LiDAR scan (/scan)")
    success, out, _ = run_cmd("timeout 1 ros2 topic info /scan 2>/dev/null")
    if success:
        print_pass()
        return True
    else:
        print_warn("Not publishing - LiDAR may not be working")
        return False

# ============================================================================
#  5. AUTO-FIX COMMON ISSUES
# ============================================================================
def auto_fix():
    print_header("Auto-Fix Common Issues")

    fixes_applied = []

    # Fix 1: Add user to dialout group
    print_test("Checking dialout group membership")
    success, out, _ = run_cmd("groups | grep dialout")
    if not success:
        print_warn("Not in dialout group - adding...")
        run_cmd("sudo usermod -a -G dialout $USER")
        fixes_applied.append("Added to dialout group (relogin required)")
        print_pass("Added (relogin required)")
    else:
        print_pass("Already in group")

    # Fix 2: Stop interfering processes
    print_test("Stopping interfering processes")
    run_cmd("pkill -f yahboom_oled")
    run_cmd("pkill -f rplidar_node")
    print_pass()
    fixes_applied.append("Stopped interfering processes")

    # Fix 3: Reset USB devices
    print_test("Resetting USB permissions")
    for dev in ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ydlidar']:
        if os.path.exists(dev):
            run_cmd(f"sudo chmod 666 {dev}")
    print_pass()
    fixes_applied.append("Reset USB permissions")

    if fixes_applied:
        print(f"\n{Colors.GREEN}Applied {len(fixes_applied)} fixes:{Colors.END}")
        for fix in fixes_applied:
            print(f"  • {fix}")

    return True

# ============================================================================
#  MAIN
# ============================================================================
def main():
    import argparse
    parser = argparse.ArgumentParser(description='Yahboom Robot Diagnostic Tool')
    parser.add_argument('--quick', action='store_true', help='Quick health check only')
    parser.add_argument('--fix', action='store_true', help='Auto-fix common issues')
    args = parser.parse_args()

    print(f"{Colors.HEADER}")
    print("="*70)
    print("  Yahboom Rosmaster R2 - Comprehensive Diagnostic")
    print("="*70)
    print(f"{Colors.END}")

    if args.fix:
        auto_fix()
        print("\n✅ Auto-fix complete! Try starting the robot now.")
        return

    # Run diagnostics
    results = {}

    results['usb'] = test_usb_devices()
    results['rosmaster'] = test_rosmaster()
    results['ydlidar'] = test_ydlidar()

    if not args.quick:
        results['ros2'] = test_ros2_topics()

    # Summary
    print_header("Diagnostic Summary")

    issues = []
    if not results.get('rosmaster'):
        issues.append("ROSMaster board not communicating")
    if not results.get('ydlidar'):
        issues.append("YDLiDAR motor not spinning")
    if not args.quick and not results.get('ros2'):
        issues.append("ROS2 system not running or incomplete")

    if not issues:
        print(f"{Colors.GREEN}✅ All systems operational!{Colors.END}\n")
        print("Ready to start robot:")
        print("  ./scripts/start_robot.sh")
    else:
        print(f"{Colors.YELLOW}⚠️  Issues found:{Colors.END}\n")
        for issue in issues:
            print(f"  • {issue}")
        print(f"\n{Colors.CYAN}Try auto-fix:{Colors.END}")
        print("  ./scripts/diagnose_robot.py --fix")

    print()

if __name__ == "__main__":
    main()
