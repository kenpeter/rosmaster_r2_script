#!/usr/bin/env python3
"""
3D World Viewer - RTAB-Map SLAM + AI Fusion Vision
Single RViz window with 3 panels: AI Fusion (LEFT) + 3D Map (RIGHT) + Info (BOTTOM)

THERMAL OPTIMIZED VERSION - Monitors temperature to prevent shutdown
"""

import os
import sys
import subprocess
import time
import signal
import threading
import argparse

# Color codes
class Colors:
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[0;33m'
    BLUE = '\033[0;34m'
    CYAN = '\033[0;36m'
    MAGENTA = '\033[0;35m'
    NC = '\033[0m'  # No Color

# Global flag for temperature monitoring
temp_warning_shown = False
shutdown_requested = False

def get_max_temperature():
    """Get the maximum temperature from all thermal zones"""
    try:
        temps = []
        with open('/sys/devices/virtual/thermal/thermal_zone0/temp', 'r') as f:
            temps.append(int(f.read().strip()) / 1000.0)  # Convert millidegrees to degrees
        with open('/sys/devices/virtual/thermal/thermal_zone1/temp', 'r') as f:
            temps.append(int(f.read().strip()) / 1000.0)
        return max(temps)
    except:
        return 0

def get_current_power_mode():
    """Get current nvpmodel power mode"""
    try:
        result = subprocess.run(['sudo', 'nvpmodel', '-q'], capture_output=True, text=True)
        # Extract mode number from output like "NV Power Mode: MAXN_SUPER\n0"
        lines = result.stdout.strip().split('\n')
        return int(lines[-1])
    except:
        return -1

def recommend_power_mode():
    """Check and recommend power mode for thermal management"""
    current_mode = get_current_power_mode()
    temp = get_max_temperature()

    print(f"{Colors.CYAN}{'=' * 70}{Colors.NC}")
    print(f"{Colors.CYAN}Thermal Check{Colors.NC}")
    print(f"{Colors.CYAN}{'=' * 70}{Colors.NC}")
    print(f"Current temperature: {Colors.YELLOW}{temp:.1f}°C{Colors.NC}")

    mode_names = {0: "MAXN_SUPER", 1: "10W", 2: "15W", 3: "25W", 4: "40W"}
    print(f"Current power mode: {Colors.YELLOW}{mode_names.get(current_mode, 'Unknown')} (ID={current_mode}){Colors.NC}")

    # Recommend power mode based on workload
    if current_mode == 0:  # MAXN_SUPER
        print(f"\n{Colors.YELLOW}⚠️  WARNING: Running in MAXN_SUPER mode!{Colors.NC}")
        print(f"{Colors.YELLOW}This workload is very intensive (RTAB-Map + YOLO11 + DINOv2 + RViz){Colors.NC}")
        print(f"\n{Colors.GREEN}RECOMMENDATION: Switch to 25W mode to prevent thermal shutdown{Colors.NC}")
        print(f"  Run: {Colors.CYAN}sudo nvpmodel -m 3{Colors.NC}")
        print(f"\nOr for even cooler operation:")
        print(f"  15W mode: {Colors.CYAN}sudo nvpmodel -m 2{Colors.NC}")
        print()

        # Check if running non-interactively or AUTO_YES env var is set
        import sys
        if not sys.stdin.isatty() or os.environ.get('AUTO_YES'):
            print(f"{Colors.YELLOW}Non-interactive mode detected - continuing automatically{Colors.NC}")
            response = 'y'
        else:
            response = input(f"{Colors.YELLOW}Continue anyway? (y/N): {Colors.NC}").strip().lower()

        if response != 'y':
            print(f"{Colors.RED}Exiting. Please switch power mode and try again.{Colors.NC}")
            return False

    print(f"{Colors.GREEN}✓ Starting with current power mode{Colors.NC}")
    print()
    return True

def monitor_temperature(stop_event):
    """Monitor temperature in background thread"""
    global temp_warning_shown, shutdown_requested

    while not stop_event.is_set():
        temp = get_max_temperature()

        # Warn at 75°C
        if temp > 75 and not temp_warning_shown:
            print(f"\n{Colors.YELLOW}⚠️  HIGH TEMPERATURE: {temp:.1f}°C{Colors.NC}")
            print(f"{Colors.YELLOW}Consider closing some processes or switching to lower power mode{Colors.NC}\n")
            temp_warning_shown = True

        # Critical at 85°C - TERMINATE SCRIPT (not shutdown machine)
        if temp > 85:
            print(f"\n{Colors.RED}🚨 CRITICAL TEMPERATURE: {temp:.1f}°C{Colors.NC}")
            print(f"{Colors.RED}⚠️  THERMAL PROTECTION: Terminating script...{Colors.NC}\n")
            shutdown_requested = True
            stop_event.set()
            break

        time.sleep(5)  # Check every 5 seconds

def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description='RTAB-Map 3D SLAM + AI Fusion Vision Viewer',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  ./show_3d_world.py                          # All features ON (default)
  ./show_3d_world.py --no-rtabmap             # Fusion vision only
  ./show_3d_world.py --no-fusion              # RTAB-Map only
  ./show_3d_world.py --no-rtabmap --no-fusion # Minimal (no heavy processing)
        '''
    )
    parser.add_argument('--no-rtabmap', action='store_true',
                        help='Disable RTAB-Map 3D SLAM')
    parser.add_argument('--no-fusion', action='store_true',
                        help='Disable AI Fusion Vision (YOLO11 + DINOv2)')
    return parser.parse_args()

def print_banner():
    """Print the startup banner"""
    print(f"{Colors.CYAN}{'=' * 70}{Colors.NC}")
    print(f"{Colors.CYAN}  RTAB-Map 3D SLAM + AI Fusion Vision{Colors.NC}")
    print(f"{Colors.CYAN}  Integrated RViz Layout{Colors.NC}")
    print(f"{Colors.CYAN}{'=' * 70}{Colors.NC}")
    print()
    print(f"{Colors.GREEN}One RViz window with 3 panels:{Colors.NC}")
    print(f"  LEFT   : AI Fusion Vision (YOLO11 + DINOv2)")
    print(f"  RIGHT  : 3D SLAM Map (Point Cloud + Path)")
    print(f"  BOTTOM : RTAB-Map Info")
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
    print(f"{Colors.GREEN}🚀 Starting RTAB-Map 3D SLAM (headless)...{Colors.NC}")
    print(f"{Colors.GREEN}{'=' * 70}{Colors.NC}")
    print()

    # Launch RTAB-Map without visualizer
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
    print(f"{Colors.BLUE}🤖 Starting AI Fusion Vision (YOLO11 + DINOv2)...{Colors.NC}")
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

    # Kill rqt_image_view since we're using RViz panel instead
    # Give it a moment to start first
    time.sleep(3)
    subprocess.run(['pkill', '-f', 'rqt_image_view.*fusion_vision'],
                   stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL)

    return process

def launch_rviz(source_cmd, script_dir):
    """Launch RViz with 3-panel layout"""
    print(f"{Colors.MAGENTA}{'=' * 70}{Colors.NC}")
    print(f"{Colors.MAGENTA}🎨 Launching RViz2 - Integrated View{Colors.NC}")
    print(f"{Colors.MAGENTA}{'=' * 70}{Colors.NC}")
    print()

    rviz_config = os.path.join(script_dir, "rtabmap_slam_fusion.rviz")
    cmd = f"{source_cmd} && rviz2 -d {rviz_config}"

    process = subprocess.Popen(
        cmd,
        shell=True,
        executable='/bin/bash'
    )

    return process

def check_topic_health(source_cmd, topic_name, expected_rate=None, timeout=5):
    """
    Check if a topic exists AND is publishing data.

    Args:
        source_cmd: ROS2 environment setup command
        topic_name: Topic to check (e.g., '/camera/color/image_raw')
        expected_rate: Expected Hz (optional, for rate checking)
        timeout: Seconds to wait for data

    Returns:
        dict: {
            'exists': bool,
            'publishing': bool,
            'rate': float or None,
            'message': str
        }
    """
    result = {
        'exists': False,
        'publishing': False,
        'rate': None,
        'message': ''
    }

    # 1. Check if topic exists
    cmd = f"{source_cmd} && ros2 topic list"
    proc = subprocess.run(cmd, shell=True, executable='/bin/bash',
                         capture_output=True, text=True, timeout=10)

    if topic_name not in proc.stdout:
        result['message'] = f"Topic {topic_name} does not exist"
        return result

    result['exists'] = True

    # 2. Check if topic is publishing (with timeout)
    cmd = f"timeout {timeout} bash -c '{source_cmd} && ros2 topic echo {topic_name} --once > /dev/null 2>&1'"
    proc = subprocess.run(cmd, shell=True, executable='/bin/bash')

    if proc.returncode == 0:
        result['publishing'] = True
    elif proc.returncode == 124:  # timeout exit code
        result['message'] = f"Topic exists but no data in {timeout}s"
        return result
    else:
        result['message'] = f"Error reading topic"
        return result

    # 3. Check publishing rate (if requested)
    if expected_rate and result['publishing']:
        cmd = f"timeout 3 bash -c '{source_cmd} && ros2 topic hz {topic_name} --window 10 2>&1'"
        proc = subprocess.run(cmd, shell=True, executable='/bin/bash',
                             capture_output=True, text=True)

        # Parse output like "average rate: 30.123"
        import re
        match = re.search(r'average rate:\s*([\d.]+)', proc.stdout)
        if match:
            result['rate'] = float(match.group(1))
            if result['rate'] < expected_rate * 0.5:  # Less than 50% of expected
                result['message'] = f"Rate too low: {result['rate']:.1f} Hz (expected ~{expected_rate} Hz)"
            else:
                result['message'] = f"OK ({result['rate']:.1f} Hz)"
        else:
            result['message'] = "Publishing (rate unknown)"
    else:
        result['message'] = "Publishing"

    return result

def check_camera_diagnostics(source_cmd):
    """
    Verify camera topics and data quality.

    Returns:
        dict: {
            'color_ok': bool,
            'depth_ok': bool,
            'points_ok': bool,
            'camera_info_ok': bool,
            'point_cloud_topic': str or None,
            'issues': list of str
        }
    """
    diag = {
        'color_ok': False,
        'depth_ok': False,
        'points_ok': False,
        'camera_info_ok': False,
        'point_cloud_topic': None,
        'issues': []
    }

    # Check RGB stream
    health = check_topic_health(source_cmd, '/camera/color/image_raw',
                                expected_rate=30, timeout=5)
    if health['publishing']:
        diag['color_ok'] = True
    else:
        diag['issues'].append(f"RGB Camera: {health['message']}")

    # Check Depth stream
    health = check_topic_health(source_cmd, '/camera/depth/image_raw',
                                expected_rate=30, timeout=5)
    if health['publishing']:
        diag['depth_ok'] = True
    else:
        diag['issues'].append(f"Depth Camera: {health['message']}")

    # Check Point Cloud - try multiple possible topics
    point_cloud_topics = [
        '/camera/depth_registered/points',
        '/camera/depth/color/points',
        '/camera/depth/points'
    ]

    for topic in point_cloud_topics:
        health = check_topic_health(source_cmd, topic, timeout=5)
        if health['publishing']:
            diag['points_ok'] = True
            diag['point_cloud_topic'] = topic
            break

    if not diag['points_ok']:
        diag['issues'].append("No point cloud topic publishing")

    # Check camera info
    health = check_topic_health(source_cmd, '/camera/color/camera_info', timeout=3)
    if health['publishing']:
        diag['camera_info_ok'] = True

    return diag

def check_rtabmap_status(source_cmd):
    """
    Check RTAB-Map node status and topic publications.

    Returns:
        dict: {
            'node_running': bool,
            'mapData_ok': bool,
            'mapGraph_ok': bool,
            'info_ok': bool,
            'issues': list of str
        }
    """
    status = {
        'node_running': False,
        'mapData_ok': False,
        'mapGraph_ok': False,
        'info_ok': False,
        'issues': []
    }

    # Check if rtabmap node is running
    try:
        cmd = f"{source_cmd} && ros2 node list"
        proc = subprocess.run(cmd, shell=True, executable='/bin/bash',
                             capture_output=True, text=True, timeout=10)

        if '/rtabmap/rtabmap' in proc.stdout or '/rtabmap' in proc.stdout:
            status['node_running'] = True
        else:
            status['issues'].append("RTAB-Map node not running")
            return status
    except Exception as e:
        status['issues'].append(f"Failed to check nodes: {e}")
        return status

    # Check critical RTAB-Map topics
    topics_to_check = [
        ('/mapData', 'mapData_ok'),
        ('/mapGraph', 'mapGraph_ok'),
        ('/info', 'info_ok'),
    ]

    for topic, status_key in topics_to_check:
        health = check_topic_health(source_cmd, topic, timeout=3)
        if health['publishing']:
            status[status_key] = True
        else:
            status['issues'].append(f"{topic}: {health['message']}")

    return status

def run_comprehensive_diagnostics(source_cmd):
    """
    Run all diagnostic checks before launching visualization.
    Prints detailed report and returns success status.

    Returns:
        bool: True if all critical systems are healthy
    """
    print(f"{Colors.CYAN}{'=' * 70}{Colors.NC}")
    print(f"{Colors.CYAN}System Diagnostics{Colors.NC}")
    print(f"{Colors.CYAN}{'=' * 70}{Colors.NC}")
    print()

    all_ok = True

    # 1. Check Camera
    print(f"{Colors.YELLOW}[1/4] Camera System...{Colors.NC}")
    cam_diag = check_camera_diagnostics(source_cmd)

    if cam_diag['color_ok']:
        print(f"  {Colors.GREEN}✓ RGB Camera publishing{Colors.NC}")
    else:
        print(f"  {Colors.RED}✗ RGB Camera NOT publishing{Colors.NC}")
        all_ok = False

    if cam_diag['depth_ok']:
        print(f"  {Colors.GREEN}✓ Depth Camera publishing{Colors.NC}")
    else:
        print(f"  {Colors.RED}✗ Depth Camera NOT publishing{Colors.NC}")
        all_ok = False

    if cam_diag['points_ok']:
        print(f"  {Colors.GREEN}✓ Point Cloud publishing at {cam_diag.get('point_cloud_topic', 'unknown')}{Colors.NC}")
    else:
        print(f"  {Colors.YELLOW}⚠  Point Cloud not detected (non-critical){Colors.NC}")

    for issue in cam_diag['issues']:
        print(f"    {Colors.YELLOW}• {issue}{Colors.NC}")
    print()

    # 2. Check LiDAR
    print(f"{Colors.YELLOW}[2/4] LiDAR System...{Colors.NC}")
    lidar_health = check_topic_health(source_cmd, '/scan', expected_rate=10, timeout=3)

    if lidar_health['publishing']:
        rate_str = f" at {lidar_health['rate']:.1f} Hz" if lidar_health['rate'] else ""
        print(f"  {Colors.GREEN}✓ LiDAR publishing{rate_str}{Colors.NC}")
    else:
        print(f"  {Colors.RED}✗ LiDAR NOT publishing{Colors.NC}")
        print(f"    {Colors.YELLOW}{lidar_health['message']}{Colors.NC}")
        all_ok = False
    print()

    # 3. Check Odometry
    print(f"{Colors.YELLOW}[3/4] Odometry System...{Colors.NC}")
    odom_health = check_topic_health(source_cmd, '/odom', expected_rate=50, timeout=3)

    if odom_health['publishing']:
        rate_str = f" at {odom_health['rate']:.1f} Hz" if odom_health['rate'] else ""
        print(f"  {Colors.GREEN}✓ Odometry publishing{rate_str}{Colors.NC}")
    else:
        print(f"  {Colors.RED}✗ Odometry NOT publishing{Colors.NC}")
        print(f"    {Colors.YELLOW}{odom_health['message']}{Colors.NC}")
        all_ok = False
    print()

    # 4. Check TF
    print(f"{Colors.YELLOW}[4/4] Transform System...{Colors.NC}")
    print(f"  {Colors.YELLOW}⚠  map → odom transform not available (will be created by RTAB-Map){Colors.NC}")
    print()

    # Summary
    print(f"{Colors.CYAN}{'=' * 70}{Colors.NC}")
    if all_ok:
        print(f"{Colors.GREEN}✅ All critical systems operational{Colors.NC}")
    else:
        print(f"{Colors.RED}⚠️  Some systems have issues - visualization may not work correctly{Colors.NC}")
    print(f"{Colors.CYAN}{'=' * 70}{Colors.NC}")
    print()

    return all_ok

def monitor_system_health(source_cmd, stop_event, args):
    """
    Background thread to continuously monitor system health.
    Runs every 10 seconds, alerts on issues.
    """
    last_alert_time = {}
    alert_cooldown = 30  # Don't spam alerts

    while not stop_event.is_set():
        time.sleep(10)

        if stop_event.is_set():
            break

        issues = []

        # Check RTAB-Map (if enabled)
        if not args.no_rtabmap:
            try:
                rtab_status = check_rtabmap_status(source_cmd)

                if not rtab_status['node_running']:
                    issues.append("RTAB-Map node died")
                elif not rtab_status['mapData_ok']:
                    issues.append("RTAB-Map not publishing map data")
            except:
                pass  # Silent in monitoring thread

        # Check Fusion Vision (if enabled)
        if not args.no_fusion:
            try:
                fusion_health = check_topic_health(source_cmd, '/fusion_vision/output', timeout=3)
                if not fusion_health['publishing']:
                    issues.append("AI Fusion Vision stopped")
            except:
                pass  # Silent in monitoring thread

        # Alert on new issues
        current_time = time.time()
        for issue in issues:
            if issue not in last_alert_time or \
               (current_time - last_alert_time[issue]) > alert_cooldown:
                print(f"\n{Colors.YELLOW}⚠️  ALERT: {issue}{Colors.NC}\n")
                last_alert_time[issue] = current_time

def main():
    """Main function"""
    global shutdown_requested

    # Parse command-line arguments
    args = parse_arguments()

    print_banner()

    # Show enabled features
    print(f"{Colors.GREEN}Enabled Features:{Colors.NC}")
    print(f"  RTAB-Map 3D SLAM: {Colors.GREEN if not args.no_rtabmap else Colors.RED}{'ON' if not args.no_rtabmap else 'OFF'}{Colors.NC}")
    print(f"  AI Fusion Vision: {Colors.GREEN if not args.no_fusion else Colors.RED}{'ON' if not args.no_fusion else 'OFF'}{Colors.NC}")
    print()

    # Thermal check and power mode recommendation
    if not recommend_power_mode():
        sys.exit(1)

    # Setup environment
    env, workspace_root, script_dir, source_cmd = setup_environment()

    # Check robot status
    if not check_robot_running(source_cmd):
        sys.exit(1)

    # Run comprehensive diagnostics
    print(f"{Colors.YELLOW}Running comprehensive diagnostics...{Colors.NC}")
    print()
    if not run_comprehensive_diagnostics(source_cmd):
        response = input(f"{Colors.YELLOW}Issues detected. Continue anyway? (y/N): {Colors.NC}").strip().lower()
        if response != 'y':
            print(f"{Colors.RED}Exiting. Please fix issues and try again.{Colors.NC}")
            sys.exit(1)

    processes = []

    # Start temperature monitoring thread
    stop_event = threading.Event()
    temp_thread = threading.Thread(target=monitor_temperature, args=(stop_event,), daemon=True)
    temp_thread.start()
    print(f"{Colors.GREEN}✓ Temperature monitoring active{Colors.NC}\n")

    try:
        # 1. Launch RTAB-Map SLAM (if enabled)
        if not args.no_rtabmap:
            rtabmap_proc = launch_rtabmap(source_cmd, workspace_root)
            processes.append(rtabmap_proc)

            print(f"{Colors.YELLOW}⏳ Waiting for RTAB-Map to initialize (5s)...{Colors.NC}")
            time.sleep(5)

            # Verify RTAB-Map started correctly
            print(f"{Colors.YELLOW}Verifying RTAB-Map initialization...{Colors.NC}")
            rtab_status = check_rtabmap_status(source_cmd)

            if rtab_status['node_running']:
                print(f"{Colors.GREEN}✓ RTAB-Map node running{Colors.NC}")

                if rtab_status['mapData_ok']:
                    print(f"{Colors.GREEN}✓ Map data publishing{Colors.NC}")
                else:
                    print(f"{Colors.YELLOW}⚠  Map data not yet available (waiting for sensor data...){Colors.NC}")

                if rtab_status['mapGraph_ok']:
                    print(f"{Colors.GREEN}✓ Map graph publishing{Colors.NC}")
            else:
                print(f"{Colors.RED}✗ RTAB-Map failed to start!{Colors.NC}")
                print(f"{Colors.RED}Check logs above for errors{Colors.NC}")
            print()
        else:
            print(f"{Colors.YELLOW}⏭️  Skipping RTAB-Map SLAM{Colors.NC}\n")

        # 2. Launch AI Fusion Vision (if enabled)
        if not args.no_fusion:
            fusion_proc = launch_fusion_vision(source_cmd, script_dir)
            processes.append(fusion_proc)

            print(f"{Colors.YELLOW}⏳ Waiting for AI models to load (8s)...{Colors.NC}")
            time.sleep(8)
        else:
            print(f"{Colors.YELLOW}⏭️  Skipping AI Fusion Vision{Colors.NC}\n")

        # 3. Launch RViz with integrated layout
        rviz_proc = launch_rviz(source_cmd, script_dir)
        processes.append(rviz_proc)

        # Start system health monitoring thread
        health_thread = threading.Thread(
            target=monitor_system_health,
            args=(source_cmd, stop_event, args),
            daemon=True
        )
        health_thread.start()
        print(f"{Colors.GREEN}✓ System health monitoring active{Colors.NC}\n")

        print(f"\n{Colors.GREEN}{'=' * 70}{Colors.NC}")
        print(f"{Colors.GREEN}✅ RViz launched with integrated view!{Colors.NC}")
        print(f"{Colors.GREEN}{'=' * 70}{Colors.NC}")
        print()

        if not args.no_fusion:
            print(f"{Colors.CYAN}LEFT PANEL: AI Fusion Vision{Colors.NC}")
            print(f"  • YOLO11 object detection (green boxes)")
            print(f"  • DINOv2 attention heatmap (color overlay)")
            print()

        if not args.no_rtabmap:
            print(f"{Colors.CYAN}MAIN VIEW: RTAB-Map 3D World{Colors.NC}")
            print(f"  • Colorful 3D point cloud (RGB from camera)")
            print(f"  • Blue robot trajectory path")
            print(f"  • Pose graph with loop closures")
            print()

        print(f"{Colors.YELLOW}💡 CONTROLS:{Colors.NC}")
        print(f"  • Rotate 3D view: Middle-click + drag")
        print(f"  • Zoom: Mouse wheel")
        print(f"  • Pan: Shift + Middle-click + drag")
        print()
        print(f"{Colors.YELLOW}Press Ctrl+C to stop{Colors.NC}\n")

        # Wait for processes or thermal shutdown
        while True:
            # Check if thermal shutdown requested
            if shutdown_requested:
                print(f"\n{Colors.RED}⚠️  Thermal shutdown initiated!{Colors.NC}")
                break

            # Check if any process has terminated
            for proc in processes:
                if proc.poll() is not None:
                    break

            time.sleep(0.5)

    except KeyboardInterrupt:
        print(f"\n\n{Colors.YELLOW}🛑 Stopping all systems...{Colors.NC}")

    except Exception as e:
        print(f"\n{Colors.RED}❌ Error: {e}{Colors.NC}")
        import traceback
        traceback.print_exc()

    finally:
        # Stop temperature monitoring
        stop_event.set()

        # Cleanup all processes
        for proc in processes:
            try:
                proc.terminate()
                proc.wait(timeout=5)
            except:
                try:
                    proc.kill()
                except:
                    pass

        if shutdown_requested:
            print(f"{Colors.RED}✅ Thermal protection: Script terminated (machine still running){Colors.NC}")
        else:
            print(f"{Colors.GREEN}✅ All systems stopped{Colors.NC}")

if __name__ == "__main__":
    main()
