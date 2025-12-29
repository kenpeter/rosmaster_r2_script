#!/usr/bin/env python3
"""
Stop Robot and Autonomous Driving System
Cleanly stops all robot processes and autonomous nodes
Merged from cleanup_robot.sh and stop_auto.py
"""

import subprocess
import time
import sys

# ANSI color codes
RED = '\033[0;31m'
GREEN = '\033[0;32m'
YELLOW = '\033[0;33m'
BLUE = '\033[0;34m'
NC = '\033[0m'  # No Color

def print_header():
    print(f"{BLUE}========================================={NC}")
    print(f"{BLUE}  Robot System Cleanup{NC}")
    print(f"{BLUE}========================================={NC}")
    print("")

def kill_process(process_name, display_name):
    """Kill a process by name, with force kill if needed"""
    # Check if process exists
    result = subprocess.run(
        ["pgrep", "-f", process_name],
        capture_output=True,
        text=True
    )

    if result.returncode == 0:  # Process found
        print(f"{YELLOW}Stopping {display_name}...{NC}")

        # Try graceful kill
        subprocess.run(["pkill", "-f", process_name], stderr=subprocess.DEVNULL)
        time.sleep(0.5)

        # Check if still running, force kill if needed
        check = subprocess.run(["pgrep", "-f", process_name], capture_output=True)
        if check.returncode == 0:
            print(f"{YELLOW}Force stopping {display_name}...{NC}")
            subprocess.run(["pkill", "-9", "-f", process_name], stderr=subprocess.DEVNULL)
            time.sleep(0.5)

        # Final check
        final = subprocess.run(["pgrep", "-f", process_name], capture_output=True)
        if final.returncode == 0:
            print(f"{RED}⚠️  Failed to stop {display_name}{NC}")
        else:
            print(f"{GREEN}✅ {display_name} stopped{NC}")

def stop_processes():
    """Kill all robot and autonomous processes"""

    print("Stopping all robot processes...")
    print("")

    # Stop ROS2 launch processes
    kill_process("yahboomcar_bringup", "Robot bringup")
    kill_process("yahboomcar_autonomous", "Autonomous system")
    kill_process("yahboomcar_nav", "Navigation")

    # Stop individual robot nodes
    kill_process("Ackman_driver_R2", "R2 Driver")
    kill_process("Mcnamu_driver_X3", "X3 Driver")
    kill_process("base_node_R2", "R2 Base node")
    kill_process("base_node_X3", "X3 Base node")
    kill_process("driver_node", "Driver node")
    kill_process("base_node", "Base node")

    # Stop sensors
    kill_process("sllidar_node", "LiDAR (SLLIDAR)")
    kill_process("rplidar_node", "LiDAR (RPLiDAR)")
    kill_process("ydlidar_direct_node", "YDLidar")
    kill_process("astra_camera_node", "Astra Camera")
    kill_process("imu_filter_madgwick", "IMU filter")

    # Stop control nodes
    kill_process("yahboom_joy", "Joystick control")
    kill_process("joy_node", "Joy node")

    # Stop localization
    kill_process("ekf_filter_node", "EKF filter")
    kill_process("robot_localization", "Robot localization")

    # Stop state publishers
    kill_process("robot_state_publisher", "Robot state publisher")
    kill_process("joint_state_publisher", "Joint state publisher")

    # Stop YOLO and vision
    kill_process("yolo_ros", "YOLO detection")
    kill_process("vlm_scene", "VLM scene understanding")

    # Stop visualization
    kill_process("rqt_image_view", "RQT image view")
    kill_process("rviz2", "RViz")
    kill_process("show_3d_world", "3D visualization")

    # Stop debug
    kill_process("debug_logger", "Debug logger")

    print("")
    print(f"{BLUE}Cleaning up system resources...{NC}")

    # Kill any zombie processes
    subprocess.run("killall -q zombie 2>/dev/null || true", shell=True)

    print("")

def verify_stopped():
    """Check if any robot processes are still running"""
    # Get list of processes, excluding grep itself
    result = subprocess.run(
        "ps aux | grep -E 'yahboomcar|sllidar|astra_camera|Ackman_driver|base_node|ydlidar' | grep -v grep | grep -v 'stop_auto.py'",
        shell=True,
        capture_output=True,
        text=True
    )

    print("Checking for remaining robot processes...")

    if not result.stdout.strip():
        print(f"{GREEN}✅ All robot processes stopped{NC}")
        return True
    else:
        # Count lines
        remaining = len([line for line in result.stdout.strip().split('\n') if line.strip()])
        print(f"{YELLOW}⚠️  Warning: {remaining} robot processes still running{NC}")
        print("")
        print("Remaining processes:")
        print(result.stdout)
        return False

def main():
    print_header()

    stop_processes()

    print(f"{GREEN}========================================={NC}")
    print(f"{GREEN}  Cleanup Complete!{NC}")
    print(f"{GREEN}========================================={NC}")
    print("")

    if verify_stopped():
        print("")
        sys.exit(0)
    else:
        print("")
        sys.exit(1)

if __name__ == "__main__":
    main()
