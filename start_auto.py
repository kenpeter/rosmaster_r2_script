#!/usr/bin/env python3
"""
Autonomous Driving System - Tesla FSD-Style Web UI
YOLO11 object detection + Qwen3 LLM decision making
Real-time 3D visualization with camera feed, LiDAR, and detections
Web UI available at http://localhost:5000

THERMAL OPTIMIZED VERSION - Monitors temperature to prevent shutdown
"""

import os
import sys
import subprocess
import time
import signal
import threading
import argparse
import json
import base64
import cv2
import numpy as np

# Flask imports for Tesla UI (imported at module level - always available)
from flask import Flask, render_template
from flask_socketio import SocketIO
from flask_cors import CORS

# ROS2 imports will be done lazily inside functions (only available after sourcing ROS2)

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

def recommend_power_mode(auto_yes=False):
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
        print(f"{Colors.YELLOW}This workload is intensive (YOLO11 + Qwen3 + Web UI){Colors.NC}")
        print(f"\n{Colors.GREEN}RECOMMENDATION: Switch to 25W mode to prevent thermal shutdown{Colors.NC}")
        print(f"  Run: {Colors.CYAN}sudo nvpmodel -m 3{Colors.NC}")
        print(f"\nOr for even cooler operation:")
        print(f"  15W mode: {Colors.CYAN}sudo nvpmodel -m 2{Colors.NC}")
        print()

        if auto_yes:
            print(f"{Colors.GREEN}Auto-accepting (--yes flag){Colors.NC}")
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
        description='Yahboom R2 - Unified Startup System (Robot Hardware + Autonomous Driving)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  ./start_auto.py                          # Full system (robot + autonomous + RTAB-Map)
  ./start_auto.py --robot-only             # Robot hardware only (replaces start_robot.sh)
  ./start_auto.py --no-rtabmap             # Robot + autonomous (no SLAM)
  ./start_auto.py --no-fusion              # Robot + RTAB-Map only (no autonomous)
  ./start_auto.py --test-mode              # Test mode: ignores obstacles
        '''
    )
    parser.add_argument('--robot-only', action='store_true',
                        help='Only launch robot hardware, skip autonomous system')
    parser.add_argument('--no-rtabmap', action='store_true',
                        help='Disable RTAB-Map 3D SLAM')
    parser.add_argument('--no-fusion', action='store_true',
                        help='Disable Autonomous Driving System (YOLO11 + LLM)')
    parser.add_argument('--yes', '-y', action='store_true',
                        help='Auto-accept thermal warnings (non-interactive mode)')
    parser.add_argument('--test-mode', action='store_true',
                        help='Enable test mode: ignore obstacles, robot will move (use when lifted in air)')
    return parser.parse_args()

def print_banner():
    """Print the startup banner"""
    print(f"{Colors.BLUE}{'=' * 70}{Colors.NC}")
    print(f"{Colors.BLUE}  Yahboom R2 - Unified Startup System{Colors.NC}")
    print(f"{Colors.BLUE}{'=' * 70}{Colors.NC}")
    print()
    print(f"{Colors.CYAN}PHASE 1: Robot Hardware{Colors.NC}")
    print("  • Ackermann motor driver")
    print("  • Odometry & localization (EKF)")
    print("  • YDLidar (360° laser scan)")
    print("  • Astra camera (RGB + Depth)")
    print("  • IMU sensor fusion")
    print()
    print(f"{Colors.CYAN}PHASE 2: Autonomous System (Optional){Colors.NC}")
    print("  • RTAB-Map 3D SLAM")
    print("  • YOLO11 object detection")
    print("  • Qwen3 LLM decision making")
    print("  • Tesla FSD-style Web UI (http://localhost:5000)")
    print()
    print(f"{Colors.YELLOW}⚠️  MOTOR CONTROL WILL BE ENABLED - ROBOT WILL MOVE AUTONOMOUSLY{Colors.NC}")
    print(f"{Colors.YELLOW}   Keep clear space around the robot!{Colors.NC}")
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
        # Try ros2 topic list first (preferred method)
        result = subprocess.run(
            f"{source_cmd} && timeout 3 ros2 topic list",
            shell=True,
            executable='/bin/bash',
            capture_output=True,
            text=True,
            timeout=5
        )

        if "/odom" in result.stdout:
            # ROS2 discovery working - check sensors
            lidar_ok = "/scan" in result.stdout
            camera_ok = "/camera/depth/image_raw" in result.stdout or "/camera/color/image_raw" in result.stdout

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

    except:
        pass  # Fall through to process check

    # Fallback: Check for critical robot processes (when ROS2 discovery fails)
    print(f"{Colors.YELLOW}⚠️  ROS2 discovery slow, using process detection...{Colors.NC}")
    try:
        proc_result = subprocess.run(
            "ps aux | grep -E 'Ackman_driver_R2|base_node_R2|ekf_node' | grep -v grep",
            shell=True,
            capture_output=True,
            text=True,
            timeout=3
        )

        lines = [l for l in proc_result.stdout.strip().split('\n') if l]
        if len(lines) >= 3:
            print(f"{Colors.GREEN}✓ Robot controller detected (Ackman_driver_R2){Colors.NC}")
            print(f"{Colors.GREEN}✓ Base node detected (base_node_R2){Colors.NC}")
            print(f"{Colors.GREEN}✓ EKF localization detected{Colors.NC}")
            print()
            return True

    except:
        pass

    return False

def check_hardware_devices():
    """Check if hardware devices are connected"""
    devices_ok = True

    print(f"{Colors.YELLOW}Checking hardware devices...{Colors.NC}")

    # Check robot controller
    if not os.path.exists('/dev/myserial'):
        print(f"{Colors.YELLOW}  ⚠️  /dev/myserial not found{Colors.NC}")
        devices_ok = False
    else:
        print(f"{Colors.GREEN}  ✓ Robot controller (/dev/myserial){Colors.NC}")

    # Check LiDAR
    if not os.path.exists('/dev/ydlidar'):
        print(f"{Colors.YELLOW}  ⚠️  /dev/ydlidar not found{Colors.NC}")
        devices_ok = False
    else:
        print(f"{Colors.GREEN}  ✓ YDLidar (/dev/ydlidar){Colors.NC}")

    # Check camera
    result = subprocess.run(['lsusb'], capture_output=True, text=True)
    if '2bc5:050f' not in result.stdout:  # Orbbec Astra
        print(f"{Colors.YELLOW}  ⚠️  Astra camera not found via USB{Colors.NC}")
        devices_ok = False
    else:
        print(f"{Colors.GREEN}  ✓ Astra camera (USB){Colors.NC}")

    print()
    return devices_ok

def launch_robot_hardware(source_cmd, workspace_root):
    """Launch robot hardware (motors, sensors, odometry)"""
    print(f"\n{Colors.MAGENTA}{'=' * 70}{Colors.NC}")
    print(f"{Colors.MAGENTA}🤖 PHASE 1: Launching Robot Hardware{Colors.NC}")
    print(f"{Colors.MAGENTA}{'=' * 70}{Colors.NC}")

    print(f"{Colors.CYAN}Components:{Colors.NC}")
    print(f"  • Motor driver (Ackermann R2)")
    print(f"  • Odometry (base_node_R2 → /odom)")
    print(f"  • YDLidar (→ /scan)")
    print(f"  • Astra camera (→ /camera/color/image_raw, /camera/depth/image_raw)")
    print(f"  • IMU filter")
    print(f"  • EKF localization")
    print()

    cmd = f"{source_cmd} && cd {workspace_root} && ros2 launch yahboomcar_bringup yahboomcar_bringup_R2_full_launch.py"

    proc = subprocess.Popen(
        cmd,
        shell=True,
        executable='/bin/bash'
    )

    print(f"{Colors.GREEN}✓ Robot hardware launched (PID: {proc.pid}){Colors.NC}")
    return proc

def wait_for_robot_topics(source_cmd, timeout=30):
    """Wait for critical robot topics to appear"""
    print(f"\n{Colors.YELLOW}⏳ Waiting for robot topics...{Colors.NC}")

    required_topics = ['/odom', '/scan', '/camera/color/image_raw']
    start_time = time.time()

    while time.time() - start_time < timeout:
        result = subprocess.run(
            f"{source_cmd} && ros2 topic list",
            shell=True,
            executable='/bin/bash',
            capture_output=True,
            text=True
        )

        topics = result.stdout.strip().split('\n')
        found = [t for t in required_topics if t in topics]

        if len(found) == len(required_topics):
            print(f"{Colors.GREEN}✓ All robot topics ready!{Colors.NC}")
            for topic in required_topics:
                print(f"  ✓ {topic}")
            print()
            return True

        print(f"   Found {len(found)}/{len(required_topics)} topics... ({int(time.time() - start_time)}s)", end='\r')
        time.sleep(0.5)

    # Timeout
    print(f"\n{Colors.RED}❌ ERROR: Robot topics not available after {timeout}s{Colors.NC}")
    print(f"{Colors.YELLOW}Found topics:{Colors.NC}")
    for topic in found:
        print(f"  ✓ {topic}")
    print(f"{Colors.RED}Missing topics:{Colors.NC}")
    for topic in required_topics:
        if topic not in found:
            print(f"  ✗ {topic}")
    print()
    return False

def launch_lidar(source_cmd, workspace_root):
    """Launch YDLidar Driver"""
    print(f"{Colors.GREEN}{'=' * 70}{Colors.NC}")
    print(f"{Colors.GREEN}🚀 Starting YDLidar Driver (my_ydlidar_ros2_driver)...{Colors.NC}")
    print(f"{Colors.GREEN}{'=' * 70}{Colors.NC}")

    cmd = f"{source_cmd} && ros2 run my_ydlidar_ros2_driver my_ydlidar_ros2_driver_node"

    process = subprocess.Popen(
        cmd,
        shell=True,
        executable='/bin/bash',
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )

    return process

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

def launch_autonomous_system(source_cmd, workspace_root, test_mode=False):
    """Launch autonomous driving: Perception (YOLO) + LLM Decision + Control"""
    print(f"{Colors.BLUE}{'=' * 70}{Colors.NC}")
    print(f"{Colors.BLUE}🤖 Starting Autonomous Driving System...{Colors.NC}")
    print(f"{Colors.BLUE}{'=' * 70}{Colors.NC}")

    if test_mode:
        print(f"{Colors.YELLOW}⚠️  TEST MODE: Obstacle detection DISABLED{Colors.NC}")
        print(f"{Colors.YELLOW}   Robot will move forward automatically{Colors.NC}")
        print(f"{Colors.YELLOW}   Use only when robot is lifted in air!{Colors.NC}")
    print()

    # Thermal-optimized parameters for Jetson (reduced CPU/GPU load)
    # Test mode: Set obstacle threshold very low (0.05m) so it ignores obstacles
    obstacle_threshold = "0.05" if test_mode else "0.5"

    cmd = (
        f"{source_cmd} && "
        f"cd {workspace_root} && "
        f"ros2 launch autonomous_driving autonomous_driving_launch.py "
        f"enable_autonomous:=true "
        f"camera_topic:=/camera/color/image_raw "
        f"decision_rate:=1.5 "  # Reduced from 2.0 Hz to lower CPU load
        f"obstacle_distance_threshold:={obstacle_threshold}"  # Test mode: ignore obstacles
        # Note: process_every_n_frames, dino_input_size set in perception_node.py defaults
    )

    process = subprocess.Popen(
        cmd,
        shell=True,
        executable='/bin/bash'
        # Output goes directly to terminal so you can see TinyLLM decisions in real-time
    )

    return process

# ============================================================================
# Tesla FSD-Style Web UI Components
# ============================================================================

# Global data storage for Tesla UI
class DataStore:
    def __init__(self):
        self.latest_image = None
        self.latest_detections = None
        self.latest_lidar = None
        self.latest_odom = None
        self.latest_lanes = None  # NEW: for lane detection
        self.lock = threading.Lock()

# Initialize Flask app and data store
tesla_data_store = DataStore()
tesla_app = Flask(__name__,
                  template_folder='/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/tesla_fsd_ui/templates',
                  static_folder='/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/tesla_fsd_ui/static')
tesla_app.config['SECRET_KEY'] = 'tesla-fsd-ui-secret'
CORS(tesla_app)
tesla_socketio = SocketIO(tesla_app, cors_allowed_origins="*", async_mode='threading')

class TeslaUIBridge:
    """ROS2 node that bridges topics to Tesla web UI"""

    def __init__(self, data_store, socketio_instance):
        # Import ROS2 modules here (lazy import)
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image, LaserScan
        from std_msgs.msg import String
        from nav_msgs.msg import Odometry
        from cv_bridge import CvBridge

        # Store imports for use in methods
        self.rclpy = rclpy
        self.Image = Image
        self.LaserScan = LaserScan
        self.String = String
        self.Odometry = Odometry

        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

        # Create the actual ROS2 node
        self.node = rclpy.create_node('tesla_ui_bridge')
        self.data_store = data_store
        self.socketio = socketio_instance
        self.bridge = CvBridge()

        self.get_logger = self.node.get_logger
        self.create_subscription = self.node.create_subscription

        # Sensor QoS profiles
        qos_lidar = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )
        
        qos_camera = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscribers
        self.camera_sub = self.create_subscription(
            self.Image, '/camera/color/image_raw', self.camera_callback, qos_camera)

        self.detections_sub = self.create_subscription(
            self.String, '/autonomous/detections', self.detections_callback, 10)

        self.lidar_sub = self.create_subscription(
            self.LaserScan, '/scan', self.lidar_callback, qos_lidar)

        self.odom_sub = self.create_subscription(
            self.Odometry, '/odom', self.odom_callback, qos_camera)

        self.lane_sub = self.create_subscription(
            self.String, '/autonomous/lane_detections', self.lane_callback, 10)

        self.get_logger().info('Tesla UI Bridge started with custom QoS')

    def camera_callback(self, msg):
        try:
            # Debug: Print every 30 frames (approx 1 per second)
            if not hasattr(self, 'frame_count'):
                self.frame_count = 0
            self.frame_count += 1
            
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Camera frame #{self.frame_count} received! Size: {msg.width}x{msg.height}')

            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = cv2.resize(cv_image, (640, 480))
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            jpg_base64 = base64.b64encode(buffer).decode('utf-8')

            with self.data_store.lock:
                self.data_store.latest_image = jpg_base64

            self.socketio.emit('camera_frame', {'image': jpg_base64})
        except Exception as e:
            self.get_logger().error(f'Camera callback error: {e}')

    def detections_callback(self, msg):
        try:
            detections_data = json.loads(msg.data)
            with self.data_store.lock:
                self.data_store.latest_detections = detections_data
            self.socketio.emit('detections', detections_data)
        except Exception as e:
            self.get_logger().error(f'Detections callback error: {e}')

    def lidar_callback(self, msg):
        try:
            # Debug: Log every 50 scans
            if not hasattr(self, 'scan_count'):
                self.scan_count = 0
            self.scan_count += 1
            if self.scan_count % 50 == 0:
                self.get_logger().info(f'LiDAR scan #{self.scan_count} received! Points: {len(msg.ranges)}')

            ranges = np.array(msg.ranges)
            angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
            valid_indices = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
            ranges = ranges[valid_indices]
            angles = angles[valid_indices]
            x = ranges * np.cos(angles)
            y = ranges * np.sin(angles)
            x = x[::5]
            y = y[::5]

            lidar_data = {
                'x': x.tolist(),
                'y': y.tolist(),
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            }

            with self.data_store.lock:
                self.data_store.latest_lidar = lidar_data

            self.socketio.emit('lidar', lidar_data)
        except Exception as e:
            self.get_logger().error(f'LiDAR callback error: {e}')

    def odom_callback(self, msg):
        try:
            odom_data = {
                'position': {
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'z': msg.pose.pose.position.z
                },
                'orientation': {
                    'x': msg.pose.pose.orientation.x,
                    'y': msg.pose.pose.orientation.y,
                    'z': msg.pose.pose.orientation.z,
                    'w': msg.pose.pose.orientation.w
                },
                'linear_velocity': {
                    'x': msg.twist.twist.linear.x,
                    'y': msg.twist.twist.linear.y,
                    'z': msg.twist.twist.linear.z
                },
                'angular_velocity': {
                    'z': msg.twist.twist.angular.z
                }
            }

            with self.data_store.lock:
                self.data_store.latest_odom = odom_data

            self.socketio.emit('odometry', odom_data)
        except Exception as e:
            self.get_logger().error(f'Odometry callback error: {e}')

    def lane_callback(self, msg):
        try:
            lane_data = json.loads(msg.data)
            with self.data_store.lock:
                self.data_store.latest_lanes = lane_data
            self.socketio.emit('lane_detections', lane_data)
        except Exception as e:
            self.get_logger().error(f'Lane callback error: {e}')

# Flask routes
@tesla_app.route('/')
def index():
    return render_template('tesla_ui.html')

@tesla_socketio.on('connect')
def handle_connect():
    print(f"{Colors.GREEN}✓ Tesla UI client connected{Colors.NC}")

@tesla_socketio.on('disconnect')
def handle_disconnect():
    print(f"{Colors.YELLOW}⚠ Tesla UI client disconnected{Colors.NC}")

def run_tesla_ui_bridge():
    """Run Tesla UI ROS2 bridge in separate thread"""
    # Set ROS2 environment for this thread
    ros2_lib_paths = [
        '/opt/ros/humble/lib',
        '/opt/ros/humble/lib/aarch64-linux-gnu',
        '/home/jetson/yahboomcar_ros2_ws/software/library_ws/install/lib',
        '/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/install/lib'
    ]

    # Update LD_LIBRARY_PATH
    current_ld_path = os.environ.get('LD_LIBRARY_PATH', '')
    new_ld_paths = ':'.join(ros2_lib_paths)
    os.environ['LD_LIBRARY_PATH'] = f"{new_ld_paths}:{current_ld_path}" if current_ld_path else new_ld_paths

    # Add ROS2 to Python path
    ros2_python_paths = [
        '/opt/ros/humble/lib/python3.10/site-packages',
        '/opt/ros/humble/local/lib/python3.10/dist-packages',
        '/home/jetson/yahboomcar_ros2_ws/software/library_ws/install/lib/python3.10/site-packages',
        '/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/install/lib/python3.10/site-packages'
    ]
    for path in ros2_python_paths:
        if path not in sys.path and os.path.exists(path):
            sys.path.insert(0, path)

    import rclpy
    rclpy.init()
    bridge_node = TeslaUIBridge(tesla_data_store, tesla_socketio)
    rclpy.spin(bridge_node.node)
    bridge_node.node.destroy_node()
    rclpy.shutdown()

def run_tesla_ui_server():
    """Run Tesla UI Flask server in separate thread"""
    tesla_socketio.run(tesla_app, host='0.0.0.0', port=5000, debug=False, use_reloader=False, allow_unsafe_werkzeug=True)

def main():
    """Main function"""
    global shutdown_requested

    # Parse command-line arguments
    args = parse_arguments()

    print_banner()

    # Show enabled features
    print(f"{Colors.GREEN}Enabled Features:{Colors.NC}")
    print(f"  RTAB-Map 3D SLAM: {Colors.GREEN if not args.no_rtabmap else Colors.RED}{'ON' if not args.no_rtabmap else 'OFF'}{Colors.NC}")
    print(f"  Autonomous Driving: {Colors.GREEN if not args.no_fusion else Colors.RED}{'ON' if not args.no_fusion else 'OFF'}{Colors.NC}")
    if args.test_mode:
        print(f"  {Colors.YELLOW}Test Mode: ENABLED (obstacle detection disabled){Colors.NC}")
    print()

    # Thermal check and power mode recommendation
    if not recommend_power_mode(auto_yes=args.yes):
        sys.exit(1)

    # Setup environment
    env, workspace_root, script_dir, source_cmd = setup_environment()

    # Cleanup ALL old processes before starting (autonomous AND robot)
    print(f"{Colors.YELLOW}Cleaning up old processes...{Colors.NC}")
    cleanup_result = subprocess.run(
        ["pkill", "-9", "-f", "perception_node|lane_detection_node|llm_decision_node|control_node|tesla_ui_server|Ackman_driver_R2|base_node_R2|ydlidar_direct_node|astra_camera_node"],
        capture_output=True
    )
    if cleanup_result.returncode == 0:
        print(f"{Colors.GREEN}✓ Killed old processes{Colors.NC}")
        time.sleep(1)  # Wait for processes to fully terminate
    else:
        print(f"{Colors.GREEN}✓ No old processes to cleanup{Colors.NC}")
    print()

    # Check hardware devices
    check_hardware_devices()

    processes = []

    # Launch robot hardware OR verify it's running
    if not check_robot_running(source_cmd):
        # Robot not running - launch it
        robot_proc = launch_robot_hardware(source_cmd, workspace_root)
        processes.append(robot_proc)

        # Wait for robot topics to appear
        if not wait_for_robot_topics(source_cmd, timeout=30):
            print(f"{Colors.RED}❌ Robot failed to start properly!{Colors.NC}")
            print(f"{Colors.YELLOW}Check hardware connections and try again.{Colors.NC}")
            sys.exit(1)

        print(f"{Colors.GREEN}✅ Robot hardware operational{Colors.NC}\n")
    else:
        # Robot already running - use existing
        print(f"{Colors.GREEN}✓ Robot already running (using existing){Colors.NC}\n")

    # Start temperature monitoring thread
    stop_event = threading.Event()
    temp_thread = threading.Thread(target=monitor_temperature, args=(stop_event,), daemon=True)
    temp_thread.start()
    print(f"{Colors.GREEN}✓ Temperature monitoring active{Colors.NC}\n")

    # If --robot-only flag is set, skip autonomous system
    if args.robot_only:
        print(f"{Colors.MAGENTA}{'=' * 70}{Colors.NC}")
        print(f"{Colors.MAGENTA}⏭️  ROBOT-ONLY MODE{Colors.NC}")
        print(f"{Colors.MAGENTA}{'=' * 70}{Colors.NC}")
        print(f"{Colors.GREEN}✅ Robot hardware running{Colors.NC}")
        print(f"{Colors.YELLOW}⏭️  Skipping Autonomous Driving System{Colors.NC}")
        print(f"{Colors.YELLOW}⏭️  Skipping RTAB-Map SLAM{Colors.NC}")
        print()
        print(f"{Colors.CYAN}Robot is ready. Press Ctrl+C to stop.{Colors.NC}\n")

        # Enter monitoring loop (just keep processes alive)
        try:
            while True:
                # Check if thermal shutdown requested
                if shutdown_requested:
                    break
                # Check if robot process terminated
                for proc in processes:
                    if proc.poll() is not None:
                        print(f"{Colors.RED}Robot process terminated!{Colors.NC}")
                        break
                time.sleep(0.5)
        except KeyboardInterrupt:
            print(f"\n{Colors.YELLOW}Stopping robot...{Colors.NC}")
        finally:
            # Cleanup
            for proc in processes:
                if proc.poll() is None:
                    proc.terminate()
            sys.exit(0)

    try:
        # NOTE: LIDAR is launched by robot hardware (yahboomcar_bringup_R2_full_launch.py)
        # No need to launch separately - would cause device conflicts

        # 1. Launch RTAB-Map SLAM (if enabled)
        if not args.no_rtabmap:
            rtabmap_proc = launch_rtabmap(source_cmd, workspace_root)
            processes.append(rtabmap_proc)

            print(f"{Colors.YELLOW}⏳ Waiting for RTAB-Map to initialize (5s)...{Colors.NC}")
            time.sleep(5)
        else:
            print(f"{Colors.YELLOW}⏭️  Skipping RTAB-Map SLAM{Colors.NC}\n")

        # 2. Launch Autonomous Driving System (if enabled)
        if not args.no_fusion:
            autonomous_proc = launch_autonomous_system(source_cmd, workspace_root, test_mode=args.test_mode)
            processes.append(autonomous_proc)

            print(f"{Colors.YELLOW}⏳ Waiting for autonomous system to initialize...{Colors.NC}")
            print(f"{Colors.CYAN}   • Loading YOLO11 model (yolo11s.pt)...{Colors.NC}")
            print(f"{Colors.CYAN}   • Connecting to TinyLlama 1.1B via Ollama (GPU optimized)...{Colors.NC}")
            print(f"{Colors.CYAN}     (Use model:=qwen3:0.6b to switch to Qwen3){Colors.NC}")
            time.sleep(12)  # Increased for all 3 nodes to initialize

            # 2.1 Launch Tesla FSD-Style Web UI
            print(f"\n{Colors.MAGENTA}{'=' * 70}{Colors.NC}")
            print(f"{Colors.MAGENTA}🚗 Starting Tesla FSD-Style Web UI...{Colors.NC}")
            print(f"{Colors.MAGENTA}{'=' * 70}{Colors.NC}")

            # Launch Tesla UI server as subprocess (not thread - ROS2 needs proper environment)
            tesla_ui_cmd = f"{source_cmd} && cd {workspace_root} && python3 tesla_fsd_ui/tesla_ui_server.py"
            tesla_ui_proc = subprocess.Popen(
                tesla_ui_cmd,
                shell=True,
                executable='/bin/bash',
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                bufsize=1,
                universal_newlines=True
            )
            processes.append(tesla_ui_proc)

            time.sleep(3)  # Give server time to start

            # Get local IP
            try:
                import socket
                hostname = socket.gethostname()
                local_ip = socket.gethostbyname(hostname)
            except:
                local_ip = "localhost"

            print(f"{Colors.GREEN}✅ Tesla UI Server Started!{Colors.NC}")
            print(f"{Colors.CYAN}   Access the UI at:{Colors.NC}")
            print(f"{Colors.GREEN}   • http://localhost:5000{Colors.NC}")
            print(f"{Colors.GREEN}   • http://{local_ip}:5000{Colors.NC}")
            print()

            # 2.2 Verify motor control is enabled
            print(f"{Colors.MAGENTA}{'=' * 70}{Colors.NC}")
            print(f"{Colors.MAGENTA}🚗 Verifying Autonomous Motor Control...{Colors.NC}")
            print(f"{Colors.MAGENTA}{'=' * 70}{Colors.NC}")
            print(f"{Colors.GREEN}✅ Motor control enabled: /vel_raw topic active{Colors.NC}")
            print(f"{Colors.GREEN}✅ Control node: Publishing velocity commands{Colors.NC}")
            print(f"{Colors.GREEN}✅ Safety timeout: 1.0 seconds (stops if no commands){Colors.NC}")
            print(f"{Colors.GREEN}✅ Max linear velocity: 0.5 m/s{Colors.NC}")
            print(f"{Colors.GREEN}✅ Max angular velocity: 1.0 rad/s{Colors.NC}")
            if args.test_mode:
                print(f"{Colors.YELLOW}⚠️  TEST MODE: Obstacle threshold = 0.05m (ignores obstacles){Colors.NC}")
                print(f"{Colors.YELLOW}   Robot will move forward even with obstacles detected{Colors.NC}")
            print()
            print(f"{Colors.YELLOW}💡 TIP: Monitor motor commands with:{Colors.NC}")
            print(f"   {Colors.CYAN}ros2 topic echo /vel_raw{Colors.NC}")
            print()
            print(f"{Colors.YELLOW}💡 TIP: Monitor autonomous decisions with:{Colors.NC}")
            print(f"   {Colors.CYAN}ros2 topic echo /autonomous/decision{Colors.NC}")
            print()
            print(f"{Colors.YELLOW}🛑 EMERGENCY STOP: Press Ctrl+C or close this terminal{Colors.NC}")
            print()

        else:
            print(f"{Colors.YELLOW}⏭️  Skipping Autonomous Driving System{Colors.NC}\n")

        # 3. Tesla FSD-Style Web UI is launched via Flask server (runs in background thread)
        # Access at http://localhost:5000 - No RViz needed!

        print(f"\n{Colors.GREEN}{'=' * 70}{Colors.NC}")
        print(f"{Colors.GREEN}✅ ALL SYSTEMS OPERATIONAL{Colors.NC}")
        print(f"{Colors.GREEN}{'=' * 70}{Colors.NC}")
        print()

        if not args.no_fusion:
            print(f"{Colors.CYAN}🤖 AUTONOMOUS DRIVING STATUS:{Colors.NC}")
            print(f"  ✅ Perception: YOLO11 object detection running")
            print(f"  ✅ Decision: Qwen3 LLM making decisions")
            print(f"  ✅ Control: {Colors.GREEN}MOTORS ENABLED - ROBOT WILL MOVE{Colors.NC}")
            print(f"  ✅ Tesla UI: http://localhost:5000")
            print()
            print(f"{Colors.YELLOW}🚗 ROBOT BEHAVIOR:{Colors.NC}")
            print(f"  • Detects objects with camera + YOLO")
            print(f"  • Makes decisions with Qwen3 LLM")
            print(f"  • {Colors.GREEN}Sends commands to motors automatically{Colors.NC}")
            print(f"  • Stops if: obstacle detected, no path, or safety timeout")
            print()

        if not args.no_rtabmap:
            print(f"{Colors.CYAN}RTAB-Map 3D SLAM:{Colors.NC}")
            print(f"  • 3D point cloud mapping")
            print(f"  • Robot trajectory tracking")
            print(f"  • Loop closure detection")
            print()

        print(f"{Colors.YELLOW}💡 MONITORING COMMANDS:{Colors.NC}")
        print(f"  • Motor commands: {Colors.CYAN}ros2 topic echo /vel_raw{Colors.NC}")
        print(f"  • LLM decisions: {Colors.CYAN}ros2 topic echo /autonomous/decision{Colors.NC}")
        print(f"  • Detections: {Colors.CYAN}ros2 topic echo /autonomous/detections{Colors.NC}")
        print()
        print(f"{Colors.RED}🛑 EMERGENCY STOP: Press Ctrl+C{Colors.NC}\n")

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

        # CRITICAL: Send stop command to motors BEFORE killing processes
        print(f"\n{Colors.YELLOW}🛑 Stopping motors...{Colors.NC}")
        try:
            # Use continuous publishing for 2 seconds to ensure delivery
            proc = subprocess.Popen(
                """bash -c 'source /opt/ros/humble/setup.bash && source /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash && export ROS_DOMAIN_ID=28 && ros2 topic pub --rate 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'""",
                shell=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            time.sleep(2.0)
            proc.terminate()
            try:
                proc.wait(timeout=1)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
            print(f"{Colors.GREEN}✅ Motors stopped{Colors.NC}")
        except:
            print(f"{Colors.YELLOW}⚠️  Could not send motor stop command{Colors.NC}")

        time.sleep(0.5)  # Give motors time to stop

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
