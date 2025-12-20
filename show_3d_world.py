#!/usr/bin/env python3
"""
3D World Viewer - RTAB-Map SLAM with YDLidar TG30 + Astra Camera Fusion
Combines sensor fusion launch and RViz visualization
"""

import os
import sys
import subprocess
import time
import signal

# Color codes
class Colors:
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[0;33m'
    BLUE = '\033[0;34m'
    CYAN = '\033[0;36m'
    MAGENTA = '\033[0;35m'
    NC = '\033[0m'  # No Color

def print_banner():
    """Print the startup banner"""
    print(f"{Colors.CYAN}{'=' * 65}{Colors.NC}")
    print(f"{Colors.CYAN}  RTAB-Map 3D SLAM - Sensor Fusion & Visualization{Colors.NC}")
    print(f"{Colors.CYAN}{'=' * 65}{Colors.NC}")
    print()
    print(f"{Colors.GREEN}Sensors:{Colors.NC}")
    print(f"  {Colors.GREEN}✓{Colors.NC} YDLidar TG30 (2D scan)")
    print(f"  {Colors.GREEN}✓{Colors.NC} Astra Plus Camera (RGB-D)")
    print(f"  {Colors.GREEN}✓{Colors.NC} Robot Odometry (EKF)")
    print(f"  {Colors.GREEN}✓{Colors.NC} IMU Data")
    print()
    print(f"{Colors.BLUE}Output Topics:{Colors.NC}")
    print(f"  • /cloud_map - Complete 3D fused map")
    print(f"  • /cloud_obstacles - Detected obstacles")
    print(f"  • /map - 2D occupancy grid")
    print()

def setup_environment():
    """Setup ROS2 environment"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_root = os.path.dirname(script_dir)

    # Source ROS2 environment
    env = os.environ.copy()

    # Build source commands (single line for subprocess)
    source_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source /home/jetson/yahboomcar_ros2_ws/software/library_ws/install/setup.bash && "
        f"cd {workspace_root} && "
        f"source install/setup.bash && "
        f"export ROS_DOMAIN_ID=28"
    )

    return env, workspace_root, script_dir, source_cmd

def check_robot_running(source_cmd):
    """Check if robot system is running"""
    print(f"{Colors.YELLOW}Checking robot status...{Colors.NC}")

    try:
        result = subprocess.run(
            f"{source_cmd} && ros2 topic list",
            shell=True,
            executable='/bin/bash',
            capture_output=True,
            text=True
        )

        if "/odom" not in result.stdout:
            print(f"{Colors.RED}ERROR: Robot not running!{Colors.NC}")
            print(f"Please start the robot first:")
            print(f"  {Colors.YELLOW}./scripts/start_robot.sh{Colors.NC}")
            return False

        # Check sensors
        lidar_ok = "/scan" in result.stdout
        camera_ok = "/camera/depth/image_raw" in result.stdout

        if lidar_ok:
            print(f"{Colors.GREEN}✓ YDLidar TG30 detected{Colors.NC}")
        else:
            print(f"{Colors.RED}✗ YDLidar not found{Colors.NC}")

        if camera_ok:
            print(f"{Colors.GREEN}✓ Astra Camera detected{Colors.NC}")
        else:
            print(f"{Colors.RED}✗ Camera not found{Colors.NC}")

        print()
        return True

    except Exception as e:
        print(f"{Colors.RED}Error checking robot status: {e}{Colors.NC}")
        return False

def launch_rtabmap(source_cmd, workspace_root):
    """Launch RTAB-Map SLAM in background"""
    print(f"{Colors.GREEN}{'=' * 65}{Colors.NC}")
    print(f"{Colors.GREEN}🚀 Starting RTAB-Map 3D SLAM...{Colors.NC}")
    print(f"{Colors.GREEN}{'=' * 65}{Colors.NC}")
    print()
    print(f"{Colors.CYAN}TIPS:{Colors.NC}")
    print(f"  • Move robot slowly for best mapping")
    print(f"  • Loop closures improve map accuracy")
    print(f"  • Database: ~/.ros/rtabmap.db")
    print()

    cmd = f"{source_cmd} && cd {workspace_root} && ros2 launch yahboomcar_bringup rtabmap_slam_launch.py"

    # Launch RTAB-Map
    process = subprocess.Popen(
        cmd,
        shell=True,
        executable='/bin/bash',
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )

    return process

def launch_rviz(source_cmd, script_dir):
    """Launch RViz with fusion config"""
    print(f"{Colors.GREEN}Launching RViz with comprehensive 3D visualization...{Colors.NC}")
    print()
    print(f"{Colors.YELLOW}RViz displays:{Colors.NC}")
    print(f"  • {Colors.YELLOW}Yellow points{Colors.NC} = YDLidar TG30 2D scan")
    print(f"  • {Colors.CYAN}RGB cloud{Colors.NC} = Camera depth point cloud")
    print(f"  • {Colors.CYAN}Cyan cloud{Colors.NC} = RTAB-Map fused 3D map")
    print(f"  • {Colors.RED}Red points{Colors.NC} = Detected obstacles")
    print(f"  • {Colors.BLUE}2D grid{Colors.NC} = Occupancy map")
    print(f"  • {Colors.GREEN}Robot model{Colors.NC} = Your robot")
    print(f"  • {Colors.MAGENTA}Image panel{Colors.NC} = Camera RGB view")
    print()

    rviz_config = os.path.join(script_dir, "rtabmap_3d_fusion.rviz")
    cmd = f"{source_cmd} && rviz2 -d {rviz_config}"

    process = subprocess.Popen(
        cmd,
        shell=True,
        executable='/bin/bash'
    )

    return process

def main():
    """Main function"""
    print_banner()

    # Setup environment
    env, workspace_root, script_dir, source_cmd = setup_environment()

    # Check robot status
    if not check_robot_running(source_cmd):
        sys.exit(1)

    processes = []

    try:
        # Always launch RTAB-Map + RViz (option 2)
        rtabmap_proc = launch_rtabmap(source_cmd, workspace_root)
        processes.append(rtabmap_proc)

        # Wait a bit for RTAB-Map to initialize
        print(f"{Colors.YELLOW}Waiting for RTAB-Map to initialize...{Colors.NC}")
        time.sleep(5)

        # Launch RViz
        rviz_proc = launch_rviz(source_cmd, script_dir)
        processes.append(rviz_proc)

        print(f"\n{Colors.GREEN}✓ Both RTAB-Map visualizer and RViz launched!{Colors.NC}")
        print(f"{Colors.YELLOW}Press Ctrl+C to stop everything{Colors.NC}\n")

        # Wait for processes
        for proc in processes:
            proc.wait()

    except KeyboardInterrupt:
        print(f"\n\n{Colors.YELLOW}Stopping...{Colors.NC}")
        for proc in processes:
            try:
                proc.terminate()
                proc.wait(timeout=5)
            except:
                proc.kill()
        print(f"{Colors.GREEN}✓ Stopped{Colors.NC}")

    except Exception as e:
        print(f"\n{Colors.RED}Error: {e}{Colors.NC}")
        for proc in processes:
            try:
                proc.kill()
            except:
                pass
        sys.exit(1)

if __name__ == "__main__":
    main()
