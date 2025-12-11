#!/usr/bin/env python3
"""
Minimal YDLidar Test Script
Uses the YDLidar SDK tof_test binary to test the lidar
"""

import subprocess
import sys
import os
import time

def main():
    print("="*60)
    print("  YDLIDAR TEST - Using tof_test binary")
    print("="*60)

    # Path to the tof_test binary
    tof_test_path = "/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/build/ydlidar-sdk/build/tof_test"

    # Check if binary exists
    if not os.path.exists(tof_test_path):
        print(f"\n❌ Error: tof_test binary not found at:")
        print(f"   {tof_test_path}")
        print("\nPlease build the YDLidar SDK first:")
        print("   cd /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/build/ydlidar-sdk")
        print("   mkdir -p build && cd build")
        print("   cmake ..")
        print("   make")
        sys.exit(1)

    # Check if binary is executable
    if not os.access(tof_test_path, os.X_OK):
        print(f"\n⚠️  Binary exists but not executable. Making it executable...")
        os.chmod(tof_test_path, 0o755)

    print(f"\n✅ Found tof_test binary at: {tof_test_path}")

    # Check for lidar device
    lidar_devices = ['/dev/ydlidar', '/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2']
    found_device = None
    for dev in lidar_devices:
        if os.path.exists(dev):
            found_device = dev
            print(f"✅ Found potential lidar device: {dev}")

    if not found_device:
        print("\n⚠️  Warning: No lidar device found at standard locations")
        print("   Will attempt to run tof_test anyway...")

    print("\n" + "-"*60)
    print("  Launching tof_test...")
    print("-"*60)
    print("\nℹ️  The tof_test program will:")
    print("   1. Try to connect to the YDLidar")
    print("   2. Start the motor")
    print("   3. Display scan data")
    print("\n⌨️  Press Ctrl+C to stop the test")
    print("\n" + "="*60 + "\n")

    time.sleep(2)

    try:
        # Run the tof_test binary
        # The binary will output directly to the terminal
        result = subprocess.run(
            [tof_test_path],
            cwd="/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/build/ydlidar-sdk/build"
        )

        if result.returncode == 0:
            print("\n✅ tof_test completed successfully!")
        else:
            print(f"\n⚠️  tof_test exited with code: {result.returncode}")

    except KeyboardInterrupt:
        print("\n\n🛑 Test stopped by user (Ctrl+C)")
        print("✅ This is normal - test ended cleanly")

    except Exception as e:
        print(f"\n❌ Error running tof_test: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

    print("\n" + "="*60)
    print("  YDLIDAR TEST COMPLETE!")
    print("="*60)
    print("\n✅ If you saw scan data, the lidar is working!")
    print("❌ If no data appeared:")
    print("   - Check lidar USB connection")
    print("   - Check lidar power (should have a spinning motor)")
    print("   - Check device permissions (sudo chmod 666 /dev/ttyUSB*)")
    print("   - Try running: ls -la /dev/ttyUSB* /dev/ydlidar")

if __name__ == "__main__":
    main()
