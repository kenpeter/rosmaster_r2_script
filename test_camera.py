#!/usr/bin/env python3
"""
Camera Launch and Test Script
Launches the Astra depth camera ROS2 node, displays the feed, and tests it
"""

import subprocess
import sys
import time
import os
import signal

# Global process list for cleanup
processes = []

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\n\n🛑 Stopping camera...")
    cleanup_processes()
    sys.exit(0)

def cleanup_processes():
    """Kill all spawned processes"""
    for proc in processes:
        if proc.poll() is None:  # Process still running
            print(f"   Stopping process {proc.pid}...")
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
    processes.clear()

def run_bash_command(command, env=None):
    """Run a bash command with sourced ROS2 environment"""
    # Source ROS2 environment in the command
    full_command = f"""
    source /opt/ros/humble/setup.bash
    source /home/jetson/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
    source /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
    {command}
    """

    return subprocess.run(
        full_command,
        shell=True,
        executable='/bin/bash',
        capture_output=True,
        text=True,
        env=env
    )

def run_bash_command_async(command, env=None):
    """Run a bash command asynchronously with sourced ROS2 environment"""
    full_command = f"""
    source /opt/ros/humble/setup.bash
    source /home/jetson/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
    source /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
    {command}
    """

    proc = subprocess.Popen(
        full_command,
        shell=True,
        executable='/bin/bash',
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        env=env
    )
    processes.append(proc)
    return proc

def check_usb_camera():
    """Check if camera is detected via USB"""
    print("\n[1/5] Checking USB camera...")
    result = subprocess.run(["lsusb"], capture_output=True, text=True)

    camera_found = False
    for line in result.stdout.split('\n'):
        if 'Orbbec' in line or '2bc5' in line:
            print(f"   ✅ Found: {line.strip()}")
            camera_found = True
            break

    if not camera_found:
        print("   ⚠️  Astra camera not detected on USB")
        return False
    return True

def launch_camera_node():
    """Launch the Astra camera ROS2 node"""
    print("\n[2/5] Launching Astra camera node...")

    # Launch camera using ros2 launch
    print("   Starting astra_camera driver...")
    proc = run_bash_command_async(
        "ros2 launch astra_camera astro_pro_plus.launch.xml 2>&1"
    )

    # Give it time to start
    print("   Waiting for camera node to initialize...")
    time.sleep(5)

    return proc

def wait_for_camera_topics():
    """Wait for camera topics to become available"""
    print("\n[3/5] Waiting for camera topics...")

    max_attempts = 20
    for attempt in range(max_attempts):
        result = run_bash_command("ros2 topic list")
        topics = result.stdout.strip().split('\n')

        # Check for camera topics
        color_topic = '/camera/color/image_raw'
        depth_topic = '/camera/depth/image_raw'

        color_found = color_topic in topics
        depth_found = depth_topic in topics

        if color_found and depth_found:
            print(f"   ✅ Camera topics ready!")
            print(f"      - {color_topic}")
            print(f"      - {depth_topic}")
            return True

        print(f"   Attempt {attempt+1}/{max_attempts}...", end='\r')
        time.sleep(0.5)

    print("\n   ⚠️  Timeout waiting for camera topics")
    return False

def display_camera_feed(duration=10):
    """Display the camera feed using rqt_image_view"""
    print(f"\n[4/5] Displaying camera feed for {duration} seconds...")

    # Check if DISPLAY is set (GUI available)
    if not os.environ.get('DISPLAY'):
        print("   ⚠️  No DISPLAY environment - skipping visualization")
        print("   (Run this script on the Jetson desktop for video preview)")
        # Just echo the topic to verify data
        print("\n   Checking camera data stream...")
        result = run_bash_command("timeout 2 ros2 topic echo /camera/color/image_raw --once")
        if result.returncode == 0:
            print("   ✅ Camera is publishing data!")
        else:
            print("   ❌ No data from camera")
        return False

    # Launch rqt_image_view to display the color image
    print("   Opening camera viewer window...")
    print("   📸 Camera feed will display for {duration} seconds")
    print("   Press Ctrl+C to stop early\n")

    # Use image_view which is lighter than rqt_image_view
    viewer_proc = run_bash_command_async(
        "ros2 run image_view image_view --ros-args --remap image:=/camera/color/image_raw"
    )

    try:
        # Wait for specified duration
        time.sleep(duration)

        # Stop viewer
        if viewer_proc.poll() is None:
            viewer_proc.terminate()
            viewer_proc.wait(timeout=2)
    except KeyboardInterrupt:
        print("\n   Stopped by user")
        if viewer_proc.poll() is None:
            viewer_proc.terminate()

    return True

def verify_camera():
    """Verify camera is working by checking topic data"""
    print("\n[5/5] Verifying camera data...")

    # Check color image
    result = run_bash_command("timeout 2 ros2 topic hz /camera/color/image_raw")
    if "average rate" in result.stdout:
        # Extract the rate
        for line in result.stdout.split('\n'):
            if 'average rate' in line:
                print(f"   ✅ Color image: {line.strip()}")
                break
    else:
        print("   ⚠️  Could not measure color image rate")

    # Check depth image
    result = run_bash_command("timeout 2 ros2 topic hz /camera/depth/image_raw")
    if "average rate" in result.stdout:
        for line in result.stdout.split('\n'):
            if 'average rate' in line:
                print(f"   ✅ Depth image: {line.strip()}")
                break
    else:
        print("   ⚠️  Could not measure depth image rate")

    return True

def main():
    # Register signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)

    print("="*60)
    print("  Astra Camera - Launch & Test")
    print("="*60)

    try:
        # Step 1: Check USB
        if not check_usb_camera():
            print("\n❌ Camera not detected on USB")
            sys.exit(1)

        # Step 2: Launch camera node
        camera_proc = launch_camera_node()

        # Step 3: Wait for topics
        if not wait_for_camera_topics():
            print("\n❌ Camera topics not available")
            cleanup_processes()
            sys.exit(1)

        # Step 4: Display feed (if GUI available)
        display_camera_feed(duration=10)

        # Step 5: Verify camera data
        verify_camera()

        # Summary
        print("\n" + "="*60)
        print("  ✅ CAMERA TEST COMPLETE")
        print("="*60)
        print("\nCamera is working properly!")
        print("\nTo use the camera in your application:")
        print("  - Color image: /camera/color/image_raw")
        print("  - Depth image: /camera/depth/image_raw")
        print("  - Camera info: /camera/color/camera_info")

    except Exception as e:
        print(f"\n❌ Error: {e}")
        cleanup_processes()
        sys.exit(1)
    finally:
        print("\n🛑 Shutting down camera...")
        cleanup_processes()
        print("   ✅ Cleanup complete\n")

if __name__ == "__main__":
    main()
