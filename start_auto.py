#!/usr/bin/env python3
"""
Start Autonomous Driving System with Full Visualization
Python version of start_autonomous_with_viz.sh
"""

import subprocess
import time
import os
import signal
import sys

# Configuration
WORKSPACE_DIR = os.path.expanduser("~/yahboomcar_ros2_ws/yahboomcar_ws")
SETUP_CMD = "source /opt/ros/humble/setup.bash && source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash"

def run_terminal(title, command, geometry=None):
    """Launch a new gnome-terminal window with the given command"""
    full_cmd = f"gnome-terminal --title=\"{title}\""
    if geometry:
        full_cmd += f" --geometry={geometry}"
    
    # Wrap the command to source environment and keep terminal open
    bash_cmd = f"{SETUP_CMD} && {command}; exec bash"
    full_cmd += f" -- bash -c '{bash_cmd}'"
    
    print(f"🖥️  Opening {title}...")
    subprocess.Popen(full_cmd, shell=True)

def cleanup_processes():
    """Kill any existing autonomous and robot processes"""
    print("\n🧹 Cleaning up existing processes...")
    subprocess.run("pkill -f 'ros2 run autonomous_driving'", shell=True)
    subprocess.run("pkill -f 'ros2 launch autonomous_driving'", shell=True)
    subprocess.run("pkill -f 'ros2 launch yahboomcar_bringup'", shell=True)
    time.sleep(2)

def main():
    print("="*71)
    print("  🚀 Starting Complete Autonomous System (Python)")
    print("="*71)
    print("")
    print("  This will start:")
    print("    🤖 Robot Hardware (motors, camera, LiDAR, IMU)")
    print("    🧠 Autonomous Driving (YOLO11, DINOv3, TinyLLM)")
    print("")
    print("  Visualization windows:")
    print("    1. Main Control Terminal (this window)")
    print("    2. YOLO11 Detections (camera view with bounding boxes)")
    print("    3. DINOv3 Features (attention visualization)")
    print("    4. TinyLLM Reasoning (decision text)")
    print("    5. 3D SLAM World (RViz)")
    print("")
    print("  To stop everything: Press Ctrl+C")
    print("="*71)
    print("")

    # Change to workspace directory
    if os.path.exists(WORKSPACE_DIR):
        os.chdir(WORKSPACE_DIR)
    
    # Cleanup first
    cleanup_processes()

    processes = []

    try:
        # 1. Launch robot hardware first (motors, camera, LiDAR, IMU)
        print("🤖 Launching robot hardware (motors, camera, LiDAR, IMU)...")
        robot_cmd = f"{SETUP_CMD} && source ~/yahboomcar_ros2_ws/software/library_ws/install/setup.bash && export ROS_DOMAIN_ID=28 && ros2 launch yahboomcar_bringup yahboomcar_bringup_R2_full_launch.py"
        robot_launch = subprocess.Popen(robot_cmd, shell=True, executable="/bin/bash")
        processes.append(robot_launch)
        print("   Waiting for robot hardware to initialize...")
        time.sleep(8)

        # 2. Launch autonomous driving nodes
        print("📡 Launching autonomous driving nodes...")
        launch_cmd = f"{SETUP_CMD} && export ROS_DOMAIN_ID=28 && ros2 launch autonomous_driving autonomous_driving_launch.py enable_autonomous:=true"
        main_launch = subprocess.Popen(launch_cmd, shell=True, executable="/bin/bash")
        processes.append(main_launch)
        time.sleep(5)

        # 2. Open YOLO detections viewer
        run_terminal(
            "YOLO Detections",
            "sleep 3 && ros2 run rqt_image_view rqt_image_view /autonomous/debug_image"
        )

        # 3. Open DINOv3 features viewer
        dino_cmd = """
        sleep 3
        if ros2 topic list | grep -q '/autonomous/dino_features'; then
            ros2 run rqt_image_view rqt_image_view /autonomous/dino_features
        else
            echo '⚠️  DINOv3 visualization topic not available yet'
            echo 'Showing detections data instead...'
            echo ''
            ros2 topic echo /autonomous/detections
        fi
        """
        run_terminal("DINOv3 Features", dino_cmd)

        # 4. Open TinyLLM reasoning viewer
        llm_cmd = """
        echo '======================================================================='
        echo '  🤖 TinyLLM Decision Reasoning'
        echo '======================================================================='
        echo ''
        echo 'Waiting for decisions...'
        echo ''
        ros2 topic echo /autonomous/decision | while IFS= read -r line; do
            if [[ $line == *'---'* ]]; then
                echo '-----------------------------------------------------------------------'
            elif [[ $line == *'action:'* ]]; then
                echo \"🎯 $line\"
            elif [[ $line == *'llm_response:'* ]]; then
                echo \"💭 $line\"
            elif [[ $line == *'linear_velocity:'* ]] || [[ $line == *'angular_velocity:'* ]]; then
                echo \"🚗 $line\"
            elif [[ $line == *'obstacle_status:'* ]]; then
                echo \"⚠️  $line\"
            else
                echo \"$line\"
            fi
        done
        """
        run_terminal("TinyLLM Reasoning", llm_cmd, geometry="120x30")

        # 5. Open 3D SLAM visualization
        # Note: calling the python script directly. Assuming it's in scripts/ 
        slam_cmd = """
        cd ~/yahboomcar_ros2_ws/yahboomcar_ws/scripts
        sleep 3
        ./show_3d_world.py
        """
        # Note: Original script called ./show_3d_foxglove.py but the file list shows show_3d_world.py
        # I'll stick to show_3d_world.py based on file list, or revert if the user insists on foxglove.
        # The prompt referenced 'show_3d_foxglove.py' in the bash content, but the file list earlier showed 'show_3d_world.py'. 
        # I will use 'show_3d_world.py' as it exists in the file list I saw.
        run_terminal("3D SLAM World", slam_cmd)

        print("")
        print("✅ All visualization windows launched!")
        print("")
        print("=======================================================================")
        print("  📊 System Status")
        print("=======================================================================")
        
        time.sleep(8)
        
        print("Active ROS2 nodes:")
        subprocess.run(f"{SETUP_CMD} && ros2 node list", shell=True, executable="/bin/bash")
        
        print("\nActive topics:")
        subprocess.run(f"{SETUP_CMD} && ros2 topic list | grep autonomous", shell=True, executable="/bin/bash")

        print("")
        print("=======================================================================")
        print("  🎮 Control Commands")
        print("=======================================================================")
        print("  • Stop everything: Press Ctrl+C")
        print("  • Disable autonomous: ros2 param set /control_node enable_autonomous false")
        print("  • Check status: ros2 topic hz /autonomous/detections")
        print("")
        print("⚠️  Robot is in AUTONOMOUS MODE - monitor carefully!")
        print("Press Ctrl+C to stop...")

        # Wait for the main launch process
        main_launch.wait()

    except KeyboardInterrupt:
        print("\n🛑 Stop signal received. Shutting down...")
    finally:
        # Kill all launch processes if they're still running
        for proc in processes:
            if proc.poll() is None:
                proc.terminate()

        # Run cleanup script/commands
        cleanup_processes()
        print("✅ System stopped.")

if __name__ == "__main__":
    main()
