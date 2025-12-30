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

def check_driver_running():
    """Check if motor driver is running"""
    result = subprocess.run(
        ["pgrep", "-f", "driver_node|Ackman_driver|Mcnamu_driver"],
        capture_output=True
    )
    return result.returncode == 0

def send_stop_command():
    """Send stop command to motors BEFORE killing processes"""
    print(f"{YELLOW}🛑 Sending STOP command to motors...{NC}")

    # Check if driver is running
    if not check_driver_running():
        print(f"{YELLOW}⚠️  No driver running - motors should already be stopped{NC}")
        return

    try:
        # Use continuous publishing (not --once) for 2 seconds
        # This ensures ROS discovery happens and messages are delivered
        print(f"{YELLOW}Publishing stop commands for 2 seconds...{NC}")

        proc = subprocess.Popen(
            """bash -c 'source /opt/ros/humble/setup.bash && source /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash && export ROS_DOMAIN_ID=28 && ros2 topic pub --rate 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'""",
            shell=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        # Let it publish for 2 seconds
        time.sleep(2.0)

        # Kill the publisher
        proc.terminate()
        try:
            proc.wait(timeout=1)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()

        print(f"{GREEN}✅ Stop commands sent (20 messages @ 10Hz){NC}")
        time.sleep(0.5)  # Give motors time to stop

    except Exception as e:
        print(f"{YELLOW}⚠️  Could not send stop command: {e}{NC}")

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

    # Stop autonomous driving nodes (consolidated script)
    kill_process("perception_node", "Perception node (YOLO+DINOv2)")
    kill_process("lane_detection_node", "Lane detection node")
    kill_process("llm_decision_node", "LLM decision node")
    kill_process("control_node", "Control node")
    kill_process("tesla_ui_server", "Tesla Web UI server")

    # Stop YOLO and vision (legacy)
    kill_process("yolo_ros", "YOLO detection")
    kill_process("vlm_scene", "VLM scene understanding")

    # Stop RTAB-Map SLAM
    kill_process("rtabmap", "RTAB-Map SLAM")
    kill_process("rgbd_odometry", "RGBD odometry")

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

    # IMPORTANT: Kill control nodes FIRST, then send stop commands
    # This prevents control_node from overriding stop commands (race condition fix)
    print(f"{YELLOW}Stopping autonomous control nodes...{NC}")
    kill_process("control_node", "Control node")
    kill_process("llm_decision_node", "LLM decision node")
    time.sleep(0.5)  # Ensure they're fully stopped

    # NOW send stop command (no one to override it)
    send_stop_command()

    # Stop remaining processes
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
