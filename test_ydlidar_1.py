#!/usr/bin/env python3
"""
YDLidar ROS2 Driver Test Script
Tests the YDLidar using the ROS2 driver (ydlidar_ros2_driver)
Can use either the built ROS2 driver or fall back to SDK
"""

import subprocess
import sys
import os
import time
import signal

# Global process handle for cleanup
driver_process = None

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\n\n🛑 Stopping test...")
    if driver_process:
        print("Terminating driver process...")
        driver_process.terminate()
        time.sleep(1)
        if driver_process.poll() is None:
            driver_process.kill()
    sys.exit(0)

def check_ros2_environment():
    """Check if ROS2 is sourced"""
    ros_distro = os.environ.get('ROS_DISTRO')

    # If not set, try to source it automatically
    if not ros_distro:
        print("⚠️  ROS2 environment not detected, attempting to source...")
        # Try to source ROS2 for subprocess commands
        ros_setup = "/opt/ros/humble/setup.bash"
        if os.path.exists(ros_setup):
            # Source and get the environment
            result = subprocess.run(
                f"bash -c 'source {ros_setup} && env'",
                shell=True,
                capture_output=True,
                text=True
            )
            if result.returncode == 0:
                # Parse environment variables
                for line in result.stdout.split('\n'):
                    if '=' in line:
                        key, value = line.split('=', 1)
                        os.environ[key] = value
                print(f"✅ ROS2 environment sourced automatically: humble")
                return True
            else:
                print("⚠️  Could not source ROS2 environment")
                return False
        else:
            print("⚠️  ROS2 environment not sourced!")
            print("\nPlease source ROS2:")
            print("   source /opt/ros/humble/setup.bash")
            return False
    print(f"✅ ROS2 environment: {ros_distro}")
    return True

def test_ros2_driver_node():
    """Test using ROS2 driver node"""
    global driver_process

    print("\n" + "="*60)
    print("  METHOD 1: ROS2 Driver Node + /scan Topic")
    print("="*60)

    # Check for installed driver
    driver_paths = [
        "/home/jetson/yahboomcar_ros2_ws/software/library_ws/install/ydlidar_ros2_driver/lib/ydlidar_ros2_driver/ydlidar_ros2_driver_node",
        "/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/install/ydlidar_ros2_driver/lib/ydlidar_ros2_driver/ydlidar_ros2_driver_node"
    ]

    driver_path = None
    for path in driver_paths:
        if os.path.exists(path):
            driver_path = path
            print(f"✅ Found ROS2 driver at: {path}")
            break

    if not driver_path:
        print("❌ ROS2 driver not found at expected locations")
        print("\nSearched:")
        for path in driver_paths:
            print(f"   {path}")
        return False

    # Source the ROS2 workspace
    library_ws_setup = "/home/jetson/yahboomcar_ros2_ws/software/library_ws/install/setup.bash"

    print(f"\n📡 Launching YDLidar ROS2 driver node...")
    print("   This will start publishing to /scan topic")
    print("   The driver will run for 15 seconds to collect data\n")

    try:
        # Launch the driver node in background with proper ROS2 environment
        # Get environment with ROS2 sourced
        result = subprocess.run(
            f"bash -c 'source /opt/ros/humble/setup.bash && source {library_ws_setup} && env'",
            shell=True,
            capture_output=True,
            text=True
        )

        # Build environment dict
        env = {}
        for line in result.stdout.split('\n'):
            if '=' in line:
                key, value = line.split('=', 1)
                env[key] = value

        # Launch driver with ROS2 environment
        driver_process = subprocess.Popen(
            [driver_path, '--ros-args', '--params-file',
             '/home/jetson/yahboomcar_ros2_ws/software/library_ws/install/ydlidar_ros2_driver/share/ydlidar_ros2_driver/params/ydlidar.yaml'],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        # Give it time to start
        print("Waiting for driver to initialize (5 seconds)...")
        time.sleep(5)

        # Check if process is still running
        if driver_process.poll() is not None:
            stdout, stderr = driver_process.communicate()
            print(f"❌ Driver process exited unexpectedly!")
            print(f"\nStdout:\n{stdout.decode()}")
            print(f"\nStderr:\n{stderr.decode()}")
            return False

        print("✅ Driver process started successfully!\n")

        # Now try to read from /scan topic
        print("📊 Checking /scan topic for laser data...")
        print("   This will monitor for 10 seconds\n")

        result = subprocess.run(
            ["ros2", "topic", "echo", "/scan", "--once"],
            capture_output=True,
            text=True,
            timeout=10,
            env=env
        )

        if result.returncode == 0 and len(result.stdout) > 100:
            print("✅ Successfully received laser scan data from /scan topic!")
            print(f"\nData preview (first 500 chars):")
            print("-" * 60)
            print(result.stdout[:500])
            print("-" * 60)

            # Try to get topic info
            print("\n📊 Topic information:")
            info_result = subprocess.run(
                ["ros2", "topic", "info", "/scan"],
                capture_output=True,
                text=True,
                timeout=3,
                env=env
            )
            print(info_result.stdout)

            # Try to get topic hz
            print("📊 Checking scan rate (5 seconds)...")
            hz_result = subprocess.run(
                ["timeout", "5", "ros2", "topic", "hz", "/scan"],
                capture_output=True,
                text=True,
                env=env
            )
            if hz_result.stdout:
                print(hz_result.stdout)

            success = True
        else:
            print("❌ No data received from /scan topic")
            print("\nTroubleshooting:")
            print("   - Check if lidar is connected: ls -la /dev/ydlidar /dev/ttyUSB*")
            print("   - Check lidar power and motor")
            print("   - Check driver logs above for errors")
            success = False

        # Clean up
        print("\n🛑 Stopping driver node...")
        driver_process.terminate()
        time.sleep(1)
        if driver_process.poll() is None:
            driver_process.kill()
        driver_process = None

        return success

    except subprocess.TimeoutExpired:
        print("⚠️  Timeout waiting for /scan data")
        if driver_process:
            driver_process.terminate()
            driver_process = None
        return False

    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        if driver_process:
            driver_process.terminate()
            driver_process = None
        return False

def test_ros2_driver_client():
    """Test using the ROS2 driver client"""
    print("\n" + "="*60)
    print("  METHOD 2: ROS2 Driver Client")
    print("="*60)

    client_paths = [
        "/home/jetson/yahboomcar_ros2_ws/software/library_ws/install/ydlidar_ros2_driver/lib/ydlidar_ros2_driver/ydlidar_ros2_driver_client",
        "/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/install/ydlidar_ros2_driver/lib/ydlidar_ros2_driver/ydlidar_ros2_driver_client"
    ]

    client_path = None
    for path in client_paths:
        if os.path.exists(path):
            client_path = path
            print(f"✅ Found ROS2 client at: {path}")
            break

    if not client_path:
        print("❌ ROS2 client not found")
        return False

    print("\nℹ️  Note: The client requires the driver node to be running")
    print("   Make sure you have started the robot or the driver node")
    print("\n📊 Running client (will timeout after 10 seconds)...\n")

    try:
        result = subprocess.run(
            [client_path],
            timeout=10,
            capture_output=True,
            text=True
        )

        if result.stdout:
            print("Client output:")
            print(result.stdout[:1000])
            return True
        else:
            print("⚠️  No output from client - driver node may not be running")
            return False

    except subprocess.TimeoutExpired:
        print("⚠️  Client timed out - this is normal if driver node isn't running")
        return False
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

def main():
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)

    print("="*60)
    print("  YDLIDAR TEST - ROS2 Driver Method")
    print("="*60)

    print("\nThis script tests YDLidar using the ROS2 driver")
    print("It will try two methods:")
    print("  1. Launch driver node and subscribe to /scan topic")
    print("  2. Use the driver client (requires driver node running)")

    # Check ROS2 environment
    if not check_ros2_environment():
        print("\n❌ Cannot proceed without ROS2 environment")
        sys.exit(1)

    # Test Method 1: ROS2 driver node
    method1_success = test_ros2_driver_node()

    time.sleep(2)

    # Test Method 2: ROS2 client (optional, only if method 1 failed)
    if not method1_success:
        print("\n⚠️  Method 1 failed, trying Method 2...")
        method2_success = test_ros2_driver_client()
    else:
        method2_success = None  # Skip method 2 if method 1 worked

    # Summary
    print("\n" + "="*60)
    print("  TEST SUMMARY")
    print("="*60)

    print(f"\nMethod 1 (Driver Node + /scan):  {'✅ PASS' if method1_success else '❌ FAIL'}")
    if method2_success is not None:
        print(f"Method 2 (Driver Client):        {'✅ PASS' if method2_success else '❌ FAIL'}")

    if method1_success:
        print("\n✅ YDLidar ROS2 driver is working!")
        print("\n🚀 To use in your application:")
        print("   ros2 launch ydlidar_ros2_driver ydlidar_launch.py")
        print("   ros2 topic echo /scan")
    else:
        print("\n❌ YDLidar ROS2 driver test failed")
        print("\n💡 Alternative test method:")
        print("   Try the SDK test: ./test_ydlidar.py")
        print("\n🔧 Troubleshooting:")
        print("   - Verify lidar connection: ls -la /dev/ydlidar /dev/ttyUSB*")
        print("   - Check permissions: sudo chmod 666 /dev/ttyUSB*")
        print("   - Verify ROS2 driver is built:")
        print("     cd /home/jetson/yahboomcar_ros2_ws/software/library_ws")
        print("     colcon build --packages-select ydlidar_ros2_driver")

if __name__ == "__main__":
    main()
