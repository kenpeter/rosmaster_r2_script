#!/usr/bin/env python3
"""
Autonomous System Test Script
Tests the autonomous driving system components
"""

import subprocess
import time
import sys

def run_command(cmd, description):
    """Run a command and return result"""
    print(f"\n{'='*70}")
    print(f"  {description}")
    print('='*70)
    try:
        result = subprocess.run(
            cmd,
            shell=True,
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            print(f"✅ {description}")
            if result.stdout.strip():
                print(f"   Output: {result.stdout.strip()[:200]}")
            return True
        else:
            print(f"❌ {description}")
            if result.stderr.strip():
                print(f"   Error: {result.stderr.strip()[:200]}")
            return False
    except subprocess.TimeoutExpired:
        print(f"⏱️  {description} - Timeout (5s)")
        return False
    except Exception as e:
        print(f"❌ {description} - {e}")
        return False

def main():
    print("="*70)
    print("  Autonomous Driving System - Component Test")
    print("="*70)
    print("\nThis script tests the autonomous driving package components\n")

    # Source ROS2 environment
    source_cmd = "source /opt/ros/humble/setup.bash && source /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash"

    tests = []

    # Test 1: Check package is built
    print("\n[1/6] Checking package installation...")
    result = run_command(
        f"{source_cmd} && ros2 pkg list | grep autonomous_driving",
        "Package 'autonomous_driving' installed"
    )
    tests.append(('Package installed', result))

    # Test 2: Check perception executable
    print("\n[2/6] Checking perception node...")
    result = run_command(
        f"{source_cmd} && ros2 pkg executables autonomous_driving | grep perception_node",
        "Perception node executable found"
    )
    tests.append(('Perception node', result))

    # Test 3: Check LLM decision executable
    print("\n[3/6] Checking LLM decision node...")
    result = run_command(
        f"{source_cmd} && ros2 pkg executables autonomous_driving | grep llm_decision_node",
        "LLM decision node executable found"
    )
    tests.append(('LLM decision node', result))

    # Test 4: Check control executable
    print("\n[4/6] Checking control node...")
    result = run_command(
        f"{source_cmd} && ros2 pkg executables autonomous_driving | grep control_node",
        "Control node executable found"
    )
    tests.append(('Control node', result))

    # Test 5: Check launch file
    print("\n[5/6] Checking launch files...")
    import os
    launch_file = "/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/install/autonomous_driving/share/autonomous_driving/launch/autonomous_driving_launch.py"
    if os.path.exists(launch_file):
        print(f"✅ Launch file exists: {launch_file}")
        tests.append(('Launch file', True))
    else:
        print(f"❌ Launch file not found: {launch_file}")
        tests.append(('Launch file', False))

    # Test 6: Check dependencies
    print("\n[6/6] Checking dependencies...")
    print("\n   Checking YOLO11...")
    try:
        from ultralytics import YOLO
        print("   ✅ Ultralytics (YOLO11) available")
        yolo_ok = True
    except ImportError:
        print("   ❌ Ultralytics not available")
        yolo_ok = False

    print("\n   Checking Ollama...")
    ollama_ok = run_command(
        "ollama list | grep tinyllama:1.1b",
        "TinyLlama model available"
    )

    tests.append(('YOLO11 dependency', yolo_ok))
    tests.append(('TinyLlama dependency', ollama_ok))

    # Summary
    print("\n" + "="*70)
    print("  TEST SUMMARY")
    print("="*70)

    passed = sum(1 for _, result in tests if result)
    total = len(tests)

    print(f"\n📊 Results: {passed}/{total} tests passed\n")

    for name, result in tests:
        status = "✅" if result else "❌"
        print(f"   {status} {name}")

    print("\n" + "="*70)

    if passed == total:
        print("  ✅ ALL TESTS PASSED")
        print("="*70)
        print("\n🚀 System is ready! To start autonomous driving:\n")
        print("   cd ~/yahboomcar_ros2_ws/yahboomcar_ws")
        print("   source install/setup.bash")
        print("   ros2 launch autonomous_driving autonomous_driving_launch.py\n")
        print("⚠️  Remember: Autonomous mode starts DISABLED for safety")
        print("   Enable with: ros2 param set /control_node enable_autonomous true\n")
        return 0
    else:
        print("  ❌ SOME TESTS FAILED")
        print("="*70)
        print("\n⚠️  Fix failing tests before running autonomous system\n")
        return 1

if __name__ == "__main__":
    sys.exit(main())
