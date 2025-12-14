#!/usr/bin/env python3
"""
ONE SCRIPT FOR FOXGLOVE 3D VISUALIZATION
=========================================
Run this to see your 3D colored world in Foxglove!
Uses RGB-D camera (no LiDAR needed!)
"""

import os
import sys
import subprocess
import time
import signal
from pathlib import Path

# Colors for terminal output
GREEN = '\033[0;32m'
YELLOW = '\033[1;33m'
BLUE = '\033[0;34m'
RED = '\033[0;31m'
BOLD = '\033[1m'
NC = '\033[0m'

WORKSPACE = Path("/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws")

# Global process handles
pointcloud_process = None
foxglove_process = None

def cleanup(signum=None, frame=None):
    """Clean up processes on exit"""
    global pointcloud_process, foxglove_process
    print(f"\n{YELLOW}Shutting down...{NC}")

    if pointcloud_process:
        pointcloud_process.terminate()
        pointcloud_process.wait(timeout=2)
    if foxglove_process:
        foxglove_process.terminate()
        foxglove_process.wait(timeout=2)

    sys.exit(0)

signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

def run_ros_cmd(cmd, timeout_sec=2, capture=True):
    """Run a ROS command with proper environment"""
    full_cmd = f"source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=28 && {cmd}"

    if capture:
        result = subprocess.run(
            full_cmd,
            shell=True,
            executable='/bin/bash',
            capture_output=True,
            text=True,
            timeout=timeout_sec
        )
        return result
    else:
        subprocess.run(
            full_cmd,
            shell=True,
            executable='/bin/bash',
            timeout=timeout_sec
        )
        return None

def print_header():
    """Print welcome banner"""
    os.system('clear')
    print(f"{BOLD}{BLUE}")
    print("╔═══════════════════════════════════════════════════════════════╗")
    print("║                                                               ║")
    print("║         🦊  3D WORLD WITH FOXGLOVE  🦊                       ║")
    print("║                                                               ║")
    print("║         Your ONE script for Foxglove visualization!          ║")
    print("║                (RGB-D camera - no LiDAR needed!)             ║")
    print("║                                                               ║")
    print("╚═══════════════════════════════════════════════════════════════╝")
    print(f"{NC}\n")

def check_camera_hardware():
    """Check if Astra camera is available"""
    print(f"{YELLOW}[1/5] Checking camera hardware...{NC}")

    result = subprocess.run(
        "lsusb | grep -i 'orbbec'",
        shell=True,
        capture_output=True,
        text=True
    )

    if result.returncode == 0:
        print(f"{GREEN}✓ Orbbec Astra camera detected{NC}")
        return True
    else:
        print(f"{RED}✗ Camera NOT detected!{NC}")
        print(f"{YELLOW}  Please check USB connection{NC}")
        return False

def start_robot():
    """Start the robot if not already running"""
    print(f"{YELLOW}Starting robot...{NC}")
    print(f"{BLUE}This will take 10-15 seconds...{NC}")

    start_cmd = f"cd {WORKSPACE}/scripts && ./start_robot.sh"
    subprocess.Popen(
        start_cmd,
        shell=True,
        executable='/bin/bash',
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

    # Wait for robot to start
    print(f"{YELLOW}Waiting for robot to initialize", end="", flush=True)
    for i in range(15):
        time.sleep(1)
        print(".", end="", flush=True)

        # Check if camera node appeared
        if i >= 5:  # Start checking after 5 seconds
            try:
                result = run_ros_cmd("timeout 2 ros2 node list 2>/dev/null", timeout_sec=4)
                if result and "/camera/camera" in result.stdout:
                    print(f" {GREEN}✓{NC}")
                    return True
            except:
                pass

    print(f" {GREEN}Done{NC}")
    time.sleep(2)  # Extra buffer
    return True

def check_camera_node():
    """Check if camera node is running"""
    print(f"\n{YELLOW}[2/5] Checking camera node...{NC}")

    try:
        result = run_ros_cmd("timeout 2 ros2 node list 2>/dev/null", timeout_sec=5)

        if result and "/camera/camera" in result.stdout:
            print(f"{GREEN}✓ Camera node is running{NC}")
            return True
        else:
            print(f"{YELLOW}⚠ Camera node NOT running{NC}")
            print(f"{BLUE}  Auto-starting robot...{NC}")
            return start_robot()
    except:
        print(f"{RED}✗ Failed to check nodes{NC}")
        return False

def check_camera_publishing():
    """Check if camera is ACTUALLY publishing data"""
    print(f"\n{YELLOW}[3/5] Testing if camera is PUBLISHING data...{NC}")

    # Test color image
    print(f"  - Color image: ", end="", flush=True)
    try:
        result = run_ros_cmd("timeout 3 ros2 topic echo /camera/color/image_raw --once >/dev/null 2>&1", timeout_sec=5)
        if result.returncode == 0:
            print(f"{GREEN}✓ PUBLISHING{NC}")
            color_ok = True
        else:
            print(f"{RED}✗ NOT PUBLISHING{NC}")
            color_ok = False
    except:
        print(f"{RED}✗ TIMEOUT{NC}")
        color_ok = False

    # Test depth image
    print(f"  - Depth image: ", end="", flush=True)
    try:
        result = run_ros_cmd("timeout 3 ros2 topic echo /camera/depth/image_raw --once >/dev/null 2>&1", timeout_sec=5)
        if result.returncode == 0:
            print(f"{GREEN}✓ PUBLISHING{NC}")
            depth_ok = True
        else:
            print(f"{RED}✗ NOT PUBLISHING{NC}")
            depth_ok = False
    except:
        print(f"{RED}✗ TIMEOUT{NC}")
        depth_ok = False

    if not color_ok or not depth_ok:
        print(f"\n{RED}Camera is not publishing! Please restart robot: ./start_robot.sh{NC}")
        return False

    print(f"{GREEN}✓ Camera is publishing data!{NC}")
    return True

def launch_pointcloud_node():
    """Launch the colored point cloud generator"""
    global pointcloud_process

    print(f"\n{YELLOW}[4/5] Launching colored point cloud generator...{NC}")

    # Kill any existing instances
    subprocess.run("pkill -f 'point_cloud_xyzrgb' 2>/dev/null", shell=True)
    time.sleep(1)

    # Launch depth_image_proc point cloud node
    cmd = """
source /opt/ros/humble/setup.bash
source /home/jetson/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
cd /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws
source install/setup.bash
export ROS_DOMAIN_ID=28

ros2 run depth_image_proc point_cloud_xyzrgb_node \
    --ros-args \
    -r rgb/image_rect_color:=/camera/color/image_raw \
    -r rgb/camera_info:=/camera/color/camera_info \
    -r depth_registered/image_rect:=/camera/depth/image_raw \
    -r depth_registered/camera_info:=/camera/depth/camera_info \
    -r points:=/camera/depth/color/points \
    -p queue_size:=10 \
    -p exact_sync:=false
"""

    pointcloud_process = subprocess.Popen(
        cmd,
        shell=True,
        executable='/bin/bash',
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

    print(f"{GREEN}✓ Point cloud node launched (PID: {pointcloud_process.pid}){NC}")
    time.sleep(2)

    # Verify it's running
    if pointcloud_process.poll() is not None:
        print(f"{RED}✗ Point cloud node crashed!{NC}")
        return False

    return True

def launch_foxglove_bridge():
    """Launch Foxglove Bridge"""
    global foxglove_process

    print(f"\n{YELLOW}[5/5] Launching Foxglove Bridge...{NC}")

    cmd = """
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=28
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
"""

    foxglove_process = subprocess.Popen(
        cmd,
        shell=True,
        executable='/bin/bash',
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

    print(f"{GREEN}✓ Foxglove Bridge launched (PID: {foxglove_process.pid}){NC}")
    time.sleep(2)

    # Verify it's running
    if foxglove_process.poll() is not None:
        print(f"{RED}✗ Foxglove Bridge crashed!{NC}")
        return False

    return True

def print_success():
    """Print success message"""
    print(f"\n{GREEN}{BOLD}═══════════════════════════════════════════════════════════{NC}")
    print(f"{GREEN}{BOLD}  SUCCESS! Foxglove is ready!{NC}")
    print(f"{GREEN}{BOLD}═══════════════════════════════════════════════════════════{NC}")

    print(f"\n{BOLD}How to connect:{NC}")
    print(f"  1. Open Foxglove Studio: {BLUE}https://app.foxglove.dev{NC}")
    print(f"     (or download desktop app: {BLUE}https://foxglove.dev/download{NC})")
    print("")
    print(f"  2. Click 'Open connection'")
    print("")
    print(f"  3. Select 'Foxglove WebSocket'")
    print("")
    print(f"  4. Enter URL: {BOLD}ws://localhost:8765{NC}")
    print(f"     (or {BOLD}ws://<robot-ip>:8765{NC} from another computer)")
    print("")
    print(f"  5. Click 'Open'")

    print(f"\n{BOLD}Visualize the 3D point cloud:{NC}")
    print(f"  • Click 'Add panel' → '3D'")
    print(f"  • In left sidebar, enable: {BLUE}/camera/depth/color/points{NC}")
    print(f"  • You'll see colored 3D point cloud!")
    print(f"  • Also try: {BLUE}/camera/color/image_raw{NC} (camera view)")

    print(f"\n{BOLD}Available topics:{NC}")
    print("  📷 /camera/color/image_raw - Color camera")
    print("  📏 /camera/depth/image_raw - Depth camera")
    print("  🌍 /camera/depth/color/points - Colored 3D point cloud")
    print("  📡 /scan - Laser scan")
    print("  🧭 /odom - Odometry")
    print("  ⚡ /imu/data - IMU sensor")

    print(f"\n{YELLOW}Press Ctrl+C to stop{NC}\n")

def main():
    print_header()

    # Step 1: Check hardware
    if not check_camera_hardware():
        sys.exit(1)

    # Step 2: Check node
    if not check_camera_node():
        sys.exit(1)

    # Step 3: Check publishing
    if not check_camera_publishing():
        sys.exit(1)

    # Step 4: Launch point cloud generator
    if not launch_pointcloud_node():
        cleanup()
        sys.exit(1)

    # Step 5: Launch Foxglove Bridge
    if not launch_foxglove_bridge():
        cleanup()
        sys.exit(1)

    # Print success
    print_success()

    # Wait for processes
    try:
        while True:
            # Check if processes are still running
            if pointcloud_process.poll() is not None:
                print(f"\n{RED}✗ Point cloud node died!{NC}")
                break
            if foxglove_process.poll() is not None:
                print(f"\n{RED}✗ Foxglove Bridge died!{NC}")
                break
            time.sleep(1)
    except KeyboardInterrupt:
        pass

    cleanup()

if __name__ == '__main__':
    main()
