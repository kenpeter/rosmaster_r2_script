#!/usr/bin/env python3
"""
Stop Autonomous Driving System
Cleanly stops all autonomous and robot processes
"""

import subprocess
import time
import sys

def print_header():
    print("=" * 71)
    print("  🛑 Stopping Autonomous System")
    print("=" * 71)
    print("")

def stop_processes():
    """Kill all autonomous and robot processes"""

    print("🧹 Stopping autonomous driving nodes...")
    subprocess.run("pkill -f 'ros2 run autonomous_driving'", shell=True)
    subprocess.run("pkill -f 'ros2 launch autonomous_driving'", shell=True)
    time.sleep(1)

    print("🤖 Stopping robot hardware...")
    subprocess.run("pkill -f 'ros2 launch yahboomcar_bringup'", shell=True)
    subprocess.run("pkill -f 'yahboomcar_base_node'", shell=True)
    subprocess.run("pkill -f 'yahboom_joy'", shell=True)
    subprocess.run("pkill -f 'Ackman_driver'", shell=True)
    subprocess.run("pkill -f 'imu_filter_madgwick'", shell=True)
    subprocess.run("pkill -f 'robot_state_publisher'", shell=True)
    subprocess.run("pkill -f 'joint_state_publisher'", shell=True)
    subprocess.run("pkill -f 'ekf_node'", shell=True)
    subprocess.run("pkill -f 'joy_node'", shell=True)
    time.sleep(1)

    print("🔦 Stopping YDLiDAR...")
    subprocess.run("pkill -f 'my_ydlidar_ros2_driver_node'", shell=True)
    subprocess.run("pkill -f 'ydlidar'", shell=True)
    time.sleep(1)

    print("📷 Stopping camera...")
    subprocess.run("pkill -f 'astra_camera'", shell=True)
    subprocess.run("pkill -f 'camera_node'", shell=True)
    time.sleep(1)

    print("🖼️  Closing visualization windows...")
    subprocess.run("pkill -f 'rqt_image_view'", shell=True)
    subprocess.run("pkill -f 'rviz2'", shell=True)
    subprocess.run("pkill -f 'show_3d_world'", shell=True)
    time.sleep(1)

    print("🔍 Stopping any debug loggers...")
    subprocess.run("pkill -f 'debug_logger'", shell=True)
    time.sleep(1)

    print("🔧 Stopping ROS2 daemon...")
    subprocess.run("pkill -f 'ros2-daemon'", shell=True)
    subprocess.run("pkill -f 'ros2cli.daemon'", shell=True)
    time.sleep(1)

def verify_stopped():
    """Check if processes are still running"""
    result = subprocess.run(
        "ps aux | grep -E 'autonomous_driving|yahboomcar|ydlidar|astra_camera|imu_filter|robot_state|joint_state|ekf_node|joy_node|ros2-daemon' | grep -v grep",
        shell=True,
        capture_output=True,
        text=True
    )

    if result.stdout.strip():
        print("")
        print("⚠️  Some processes may still be running:")
        print(result.stdout)
        print("")
        print("Run this command to force kill:")
        print("  pkill -9 -f ros2")
        return False
    else:
        return True

def main():
    print_header()

    stop_processes()

    print("")
    print("Verifying shutdown...")
    time.sleep(2)

    if verify_stopped():
        print("")
        print("✅ All processes stopped successfully!")
        print("")
        print("To restart:")
        print("  ./scripts/start_auto.py    - Start autonomous system")
        print("  ./scripts/start_debug.py   - Start with debug logging")
        print("")
    else:
        sys.exit(1)

if __name__ == "__main__":
    main()
