#!/usr/bin/env python3
"""
3D World Viewer - RTAB-Map SLAM + AI Fusion Vision
Combines SLAM mapping with YOLO11 + DINOv2/v3 object detection
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
    print(f"{Colors.CYAN}{'=' * 70}{Colors.NC}")
    print(f"{Colors.CYAN}  RTAB-Map 3D SLAM + AI Fusion Vision{Colors.NC}")
    print(f"{Colors.CYAN}{'=' * 70}{Colors.NC}")
    print()
    print(f"{Colors.GREEN}Sensors:{Colors.NC}")
    print(f"  {Colors.GREEN}✓{Colors.NC} YDLidar TG30 (2D scan)")
    print(f"  {Colors.GREEN}✓{Colors.NC} Astra Plus Camera (RGB-D)")
    print(f"  {Colors.GREEN}✓{Colors.NC} Robot Odometry (EKF)")
    print(f"  {Colors.GREEN}✓{Colors.NC} IMU Data")
    print()
    print(f"{Colors.BLUE}AI Vision:{Colors.NC}")
    print(f"  {Colors.BLUE}✓{Colors.NC} YOLO11 - Object Detection")
    print(f"  {Colors.BLUE}✓{Colors.NC} DINOv2/v3 - Attention Heatmap")
    print()
    print(f"{Colors.MAGENTA}Visualization:{Colors.NC}")
    print(f"  • 3D Point Cloud Map (colorful SLAM map)")
    print(f"  • Robot Path Tracking (blue trajectory)")
    print(f"  • AI Fusion Vision Panel (YOLO + DINO)")
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
    """Launch RTAB-Map SLAM in headless mode (no visualizer)"""
    print(f"{Colors.GREEN}{'=' * 70}{Colors.NC}")
    print(f"{Colors.GREEN}🚀 Starting RTAB-Map 3D SLAM (headless mode)...{Colors.NC}")
    print(f"{Colors.GREEN}{'=' * 70}{Colors.NC}")
    print()
    print(f"{Colors.CYAN}SLAM Tips:{Colors.NC}")
    print(f"  • Move robot slowly for best mapping quality")
    print(f"  • Return to start for loop closures (better accuracy)")
    print(f"  • Database: ~/.ros/rtabmap.db")
    print()

    # Launch RTAB-Map without its visualizer
    cmd = f"{source_cmd} && cd {workspace_root} && ros2 launch yahboomcar_bringup rtabmap_slam_launch.py"

    process = subprocess.Popen(
        cmd,
        shell=True,
        executable='/bin/bash',
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )

    return process

def launch_fusion_vision(source_cmd, script_dir):
    """Launch YOLO11 + DINOv2/v3 fusion vision"""
    print(f"{Colors.BLUE}{'=' * 70}{Colors.NC}")
    print(f"{Colors.BLUE}🤖 Starting AI Fusion Vision (YOLO11 + DINOv2/v3)...{Colors.NC}")
    print(f"{Colors.BLUE}{'=' * 70}{Colors.NC}")
    print()

    fusion_script = os.path.join(script_dir, "test_fusion_live.py")
    cmd = f"{source_cmd} && python3 {fusion_script}"

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
    """Launch RViz with SLAM + AI fusion config"""
    print(f"{Colors.MAGENTA}{'=' * 70}{Colors.NC}")
    print(f"{Colors.MAGENTA}🎨 Launching RViz2 - Integrated Visualization{Colors.NC}")
    print(f"{Colors.MAGENTA}{'=' * 70}{Colors.NC}")
    print()
    print(f"{Colors.YELLOW}Main 3D View:{Colors.NC}")
    print(f"  • {Colors.CYAN}Colorful point cloud{Colors.NC} = 3D SLAM map")
    print(f"  • {Colors.BLUE}Blue path{Colors.NC} = Robot trajectory")
    print(f"  • {Colors.YELLOW}Yellow points{Colors.NC} = LiDAR scan")
    print(f"  • {Colors.GREEN}Robot model{Colors.NC} = Your robot")
    print()
    print(f"{Colors.YELLOW}AI Fusion Vision Panel:{Colors.NC}")
    print(f"  • {Colors.GREEN}Green boxes{Colors.NC} = YOLO11 object detection")
    print(f"  • {Colors.RED}Red/Yellow heatmap{Colors.NC} = DINOv2/v3 attention")
    print(f"  • Shows what AI 'sees' as important")
    print()

    rviz_config = os.path.join(script_dir, "rtabmap_slam_fusion.rviz")
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
        # 1. Launch RTAB-Map SLAM (headless)
        rtabmap_proc = launch_rtabmap(source_cmd, workspace_root)
        processes.append(rtabmap_proc)

        # Wait for RTAB-Map to initialize
        print(f"{Colors.YELLOW}⏳ Waiting for RTAB-Map to initialize...{Colors.NC}")
        time.sleep(5)

        # 2. Launch AI Fusion Vision
        fusion_proc = launch_fusion_vision(source_cmd, script_dir)
        processes.append(fusion_proc)

        # Wait for AI models to load
        print(f"{Colors.YELLOW}⏳ Waiting for AI models to load...{Colors.NC}")
        time.sleep(8)

        # 3. Launch RViz with integrated view
        rviz_proc = launch_rviz(source_cmd, script_dir)
        processes.append(rviz_proc)

        print(f"\n{Colors.GREEN}{'=' * 70}{Colors.NC}")
        print(f"{Colors.GREEN}✅ All systems launched successfully!{Colors.NC}")
        print(f"{Colors.GREEN}{'=' * 70}{Colors.NC}")
        print()
        print(f"{Colors.CYAN}Running:{Colors.NC}")
        print(f"  1. RTAB-Map 3D SLAM")
        print(f"  2. YOLO11 + DINOv2/v3 AI Fusion Vision")
        print(f"  3. RViz2 Integrated Visualization")
        print()
        print(f"{Colors.YELLOW}💡 TIP: In RViz2, you can:{Colors.NC}")
        print(f"  • Rotate 3D view with mouse")
        print(f"  • Resize the AI Fusion Vision panel")
        print(f"  • Watch the blue path track your robot")
        print(f"  • See objects detected by YOLO11 with green boxes")
        print()
        print(f"{Colors.YELLOW}Press Ctrl+C to stop everything{Colors.NC}\n")

        # Wait for processes
        for proc in processes:
            proc.wait()

    except KeyboardInterrupt:
        print(f"\n\n{Colors.YELLOW}🛑 Stopping all systems...{Colors.NC}")
        for proc in processes:
            try:
                proc.terminate()
                proc.wait(timeout=5)
            except:
                proc.kill()
        print(f"{Colors.GREEN}✅ All systems stopped{Colors.NC}")

    except Exception as e:
        print(f"\n{Colors.RED}❌ Error: {e}{Colors.NC}")
        import traceback
        traceback.print_exc()
        for proc in processes:
            try:
                proc.kill()
            except:
                pass
        sys.exit(1)

if __name__ == "__main__":
    main()
