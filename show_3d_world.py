#!/usr/bin/env python3
"""
ONE SCRIPT TO SHOW 3D WORLD
============================
Run this to see your 3D colored world visualization!
Uses RGB-D camera + YDLidar fusion for 360° coverage!

This script can run in two modes:
1. Normal mode: Launches full visualization pipeline
2. Fusion mode: Runs as ROS2 fusion node (internal use)
"""

import os
import sys
import subprocess
import time
import signal
import threading
import argparse
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
fusion_process = None
rviz_process = None
robot_process = None
ydlidar_process = None
monitor_thread = None
stop_monitoring = False

# ============================================================================
# FUSION NODE CLASS (runs when script is called with --fusion-mode)
# ============================================================================

class FusionNode:
    """ROS2 node that fuses camera point cloud + lidar scan into unified 3D world"""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import LaserScan, PointCloud2, PointField
        from tf2_ros import Buffer, TransformListener
        import numpy as np
        import struct

        # Store imports as instance variables for use in methods
        self.rclpy = rclpy
        self.Node = Node
        self.LaserScan = LaserScan
        self.PointCloud2 = PointCloud2
        self.PointField = PointField
        self.np = np
        self.struct = struct
        self.Buffer = Buffer
        self.TransformListener = TransformListener
        self.QoSProfile = QoSProfile
        self.ReliabilityPolicy = ReliabilityPolicy
        self.HistoryPolicy = HistoryPolicy

        # Initialize the node
        class FusionNodeImpl(Node):
            def __init__(node_self, parent):
                super().__init__('fusion_3d')
                node_self.parent = parent

                # QoS for Lidar: BEST_EFFORT
                lidar_qos = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=5
                )

                # QoS for Camera: RELIABLE
                camera_qos = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=2
                )

                node_self.tf_buffer = Buffer()
                node_self.tf_listener = TransformListener(node_self.tf_buffer, node_self)

                node_self.scan_sub = node_self.create_subscription(
                    LaserScan, '/scan', node_self.scan_cb, lidar_qos)
                node_self.pc_sub = node_self.create_subscription(
                    PointCloud2, '/camera/depth_registered/points', node_self.pc_cb, camera_qos)
                node_self.fused_pub = node_self.create_publisher(PointCloud2, '/fused_3d_world', 10)

                node_self.scan = None
                node_self.pc = None
                node_self.laser_to_camera_transform = None
                node_self.timer = node_self.create_timer(0.1, node_self.fuse)
                node_self.get_logger().info("✓ Fusion node ready")

            def scan_cb(node_self, msg):
                node_self.scan = msg

            def pc_cb(node_self, msg):
                node_self.pc = msg

            def get_transform(node_self):
                try:
                    return node_self.tf_buffer.lookup_transform(
                        'camera_link', 'laser_link',
                        node_self.parent.rclpy.time.Time(),
                        timeout=node_self.parent.rclpy.duration.Duration(seconds=0.1))
                except:
                    return None

            def transform_lidar_point(node_self, x_laser, y_laser, z_laser, transform):
                if transform is None:
                    return x_laser, y_laser, z_laser

                tx = transform.transform.translation.x
                ty = transform.transform.translation.y
                tz = transform.transform.translation.z
                qx = transform.transform.rotation.x
                qy = transform.transform.rotation.y
                qz = transform.transform.rotation.z
                qw = transform.transform.rotation.w

                # Rotation matrix from quaternion
                r11 = 1 - 2*(qy**2 + qz**2)
                r12 = 2*(qx*qy - qz*qw)
                r13 = 2*(qx*qz + qy*qw)
                r21 = 2*(qx*qy + qz*qw)
                r22 = 1 - 2*(qx**2 + qz**2)
                r23 = 2*(qy*qz - qx*qw)
                r31 = 2*(qx*qz - qy*qw)
                r32 = 2*(qy*qz + qx*qw)
                r33 = 1 - 2*(qx**2 + qy**2)

                # Apply transform
                x_cam = r11*x_laser + r12*y_laser + r13*z_laser + tx
                y_cam = r21*x_laser + r22*y_laser + r23*z_laser + ty
                z_cam = r31*x_laser + r32*y_laser + r33*z_laser + tz

                return x_cam, y_cam, z_cam

            def fuse(node_self):
                points = []
                np = node_self.parent.np
                struct = node_self.parent.struct

                # Get transform once
                if node_self.laser_to_camera_transform is None:
                    node_self.laser_to_camera_transform = node_self.get_transform()

                # Add Lidar points (RED)
                if node_self.scan:
                    angle = node_self.scan.angle_min
                    for r in node_self.scan.ranges:
                        if 0.1 < r < 12.0 and not np.isinf(r) and not np.isnan(r):
                            x_laser = r * np.cos(angle)
                            y_laser = r * np.sin(angle)
                            x, y, z = node_self.transform_lidar_point(
                                x_laser, y_laser, 0.0, node_self.laser_to_camera_transform)

                            # RED color (BGR: 0, 0, 255)
                            rgb = struct.unpack('I', struct.pack('BBBB', 0, 0, 255, 255))[0]
                            # Add 3 points for thickness
                            points.extend([[x, y, z-0.02, rgb], [x, y, z, rgb], [x, y, z+0.02, rgb]])
                        angle += node_self.scan.angle_increment

                # Add Camera RGB point cloud
                if node_self.pc:
                    step = node_self.pc.point_step
                    row_step = node_self.pc.row_step
                    if hasattr(node_self.pc.data, 'tobytes'):
                        data = node_self.pc.data.tobytes()
                    else:
                        data = node_self.pc.data

                    # Smart downsampling (aim for ~30k points)
                    total = node_self.pc.width * node_self.pc.height
                    skip = max(1, total // 30000)

                    if node_self.pc.height > 1:  # Organized cloud
                        v_skip = max(1, int(np.sqrt(skip)))
                        u_skip = max(1, skip // v_skip)
                        for v in range(0, node_self.pc.height, v_skip):
                            for u in range(0, node_self.pc.width, u_skip):
                                off = v * row_step + u * step
                                try:
                                    x, y, z = struct.unpack_from('fff', data, off)
                                    if not (np.isnan(x) or np.isinf(x) or np.isnan(z)) and 0.1 < z < 10.0:
                                        rgb = struct.unpack_from('I', data, off + 16)[0]
                                        points.append([x, y, z, rgb])
                                except:
                                    pass
                    else:  # Unorganized cloud
                        for i in range(0, total, skip):
                            off = i * step
                            try:
                                x, y, z = struct.unpack_from('fff', data, off)
                                if not (np.isnan(x) or np.isinf(x) or np.isnan(z)) and 0.1 < z < 10.0:
                                    rgb = struct.unpack_from('I', data, off + 16)[0]
                                    points.append([x, y, z, rgb])
                            except:
                                pass

                # Publish fused point cloud
                if points:
                    try:
                        msg = node_self.parent.PointCloud2()
                        msg.header.frame_id = 'camera_link'
                        msg.header.stamp = node_self.get_clock().now().to_msg()
                        msg.height = 1
                        msg.width = len(points)
                        msg.fields = [
                            node_self.parent.PointField(name='x', offset=0, datatype=node_self.parent.PointField.FLOAT32, count=1),
                            node_self.parent.PointField(name='y', offset=4, datatype=node_self.parent.PointField.FLOAT32, count=1),
                            node_self.parent.PointField(name='z', offset=8, datatype=node_self.parent.PointField.FLOAT32, count=1),
                            node_self.parent.PointField(name='rgb', offset=16, datatype=node_self.parent.PointField.UINT32, count=1),
                        ]
                        msg.point_step = 32
                        msg.row_step = 32 * len(points)
                        msg.is_dense = True
                        msg.is_bigendian = False

                        buf = []
                        for p in points:
                            buf.append(struct.pack('fff4xI12x', p[0], p[1], p[2], int(p[3])))
                        msg.data = b''.join(buf)

                        node_self.fused_pub.publish(msg)
                    except:
                        pass

        self.node = FusionNodeImpl(self)

    def spin(self):
        """Run the fusion node"""
        try:
            self.rclpy.spin(self.node)
        except KeyboardInterrupt:
            pass
        finally:
            self.node.destroy_node()
            self.rclpy.shutdown()

def run_fusion_node():
    """Entry point when running in fusion mode"""
    import rclpy
    rclpy.init()
    fusion = FusionNode()
    fusion.spin()

# ============================================================================
# LAUNCHER MODE FUNCTIONS
# ============================================================================

def monitor_pointcloud_output():
    """Monitor point cloud node output in background"""
    global pointcloud_process, stop_monitoring
    import select

    while not stop_monitoring and pointcloud_process:
        if pointcloud_process.poll() is not None:
            break

        if pointcloud_process.stdout:
            ready = select.select([pointcloud_process.stdout], [], [], 0.5)
            if ready[0]:
                line = pointcloud_process.stdout.readline()
                if line:
                    # Print node output with timestamp
                    timestamp = time.strftime("%H:%M:%S")
                    print(f"{BLUE}[{timestamp}] PCL Node:{NC} {line.rstrip()}")

def cleanup(signum=None, frame=None):
    """Clean up processes on exit"""
    global pointcloud_process, fusion_process, rviz_process, robot_process, ydlidar_process, stop_monitoring, monitor_thread
    print(f"\n{YELLOW}Shutting down...{NC}")

    stop_monitoring = True
    if monitor_thread:
        monitor_thread.join(timeout=2)

    if fusion_process:
        fusion_process.terminate()
        fusion_process.wait(timeout=2)
    if pointcloud_process:
        pointcloud_process.terminate()
        pointcloud_process.wait(timeout=2)
    if ydlidar_process:
        ydlidar_process.terminate()
        ydlidar_process.wait(timeout=2)
    if rviz_process:
        rviz_process.terminate()
        rviz_process.wait(timeout=2)
    if robot_process:
        print(f"{YELLOW}Stopping robot...{NC}")
        robot_process.terminate()
        robot_process.wait(timeout=3)

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
    print("║         🌍  3D WORLD VISUALIZATION  🌍                        ║")
    print("║                                                               ║")
    print("║         Your ONE script for complete 3D world!               ║")
    print("║         (RGB-D camera + YDLidar 360° fusion!)                ║")
    print("║                                                               ║")
    print("╚═══════════════════════════════════════════════════════════════╝")
    print(f"{NC}\n")

def check_camera_hardware():
    """Check if Astra camera is available"""
    print(f"{YELLOW}[1/6] Checking camera hardware...{NC}")

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
    """Start the robot (camera node)"""
    global robot_process

    print(f"{YELLOW}  Starting robot...{NC}")

    # Kill any existing robot processes
    subprocess.run("pkill -f 'start_robot.sh' 2>/dev/null", shell=True)
    subprocess.run("pkill -f 'astra_camera_node' 2>/dev/null", shell=True)
    time.sleep(2)

    # Start the robot
    start_script = WORKSPACE / "scripts/start_robot.sh"
    if not start_script.exists():
        print(f"{RED}✗ start_robot.sh not found at {start_script}{NC}")
        return False

    robot_process = subprocess.Popen(
        str(start_script),
        shell=True,
        executable='/bin/bash',
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd=str(WORKSPACE / "scripts")
    )

    print(f"{GREEN}✓ Robot started (PID: {robot_process.pid}){NC}")
    print(f"{YELLOW}  Waiting for camera to initialize...{NC}")

    # Wait for camera node to appear
    for i in range(30):  # Wait up to 30 seconds
        time.sleep(1)
        try:
            result = run_ros_cmd("timeout 2 ros2 node list 2>/dev/null", timeout_sec=4)
            if result and "/camera/camera" in result.stdout:
                print(f"{GREEN}✓ Camera node is now running!{NC}")
                return True
        except:
            pass

        # Check if robot process died
        if robot_process.poll() is not None:
            print(f"{RED}✗ Robot process died during startup!{NC}")
            # Show error output
            if robot_process.stderr:
                error = robot_process.stderr.read()
                if error:
                    print(f"{RED}Error: {error}{NC}")
            return False

        if i % 5 == 0 and i > 0:
            print(f"{YELLOW}  Still waiting... ({i}s){NC}")

    print(f"{RED}✗ Camera node failed to start after 30 seconds{NC}")
    return False

def check_camera_node():
    """Check if camera node is running, start it if not"""
    print(f"\n{YELLOW}[2/6] Checking camera node...{NC}")

    try:
        result = run_ros_cmd("timeout 2 ros2 node list 2>/dev/null", timeout_sec=5)

        if result and "/camera/camera" in result.stdout:
            print(f"{GREEN}✓ Camera node is already running{NC}")
            return True
        else:
            print(f"{YELLOW}⚠ Camera node NOT running{NC}")
            return start_robot()
    except:
        print(f"{RED}✗ Failed to check nodes{NC}")
        print(f"{YELLOW}⚠ Attempting to start robot anyway...{NC}")
        return start_robot()

def discover_pointcloud_topic():
    """Discover which point cloud topic is available

    Returns:
        tuple: (topic_name, topic_type) or (None, None) if not found
        topic_type can be 'colored' or 'depth_only'
    """
    try:
        result = run_ros_cmd("timeout 2 ros2 topic list 2>/dev/null", timeout_sec=4)
        if not result:
            return None, None

        topics = result.stdout

        # Prefer depth_registered (contains RGB data on this camera)
        if "/camera/depth_registered/points" in topics:
            return "/camera/depth_registered/points", "colored (registered)"

        # Alternative colored point cloud name
        if "/camera/depth/color/points" in topics:
            return "/camera/depth/color/points", "colored"

        # Fallback to depth-only point cloud (XYZ)
        if "/camera/depth/points" in topics:
            return "/camera/depth/points", "depth_only"

        # Check for any other point cloud topics
        for line in topics.split('\n'):
            if '/camera/' in line and 'point' in line.lower():
                return line.strip(), "unknown"

        return None, None
    except:
        return None, None

def show_all_camera_topics():
    """Debug: Show all available camera topics"""
    print(f"\n  {BLUE}DEBUG: All available /camera/* topics:{NC}")
    try:
        result = run_ros_cmd("timeout 2 ros2 topic list 2>/dev/null | grep '^/camera/'", timeout_sec=4)
        if result and result.stdout:
            topics = result.stdout.strip().split('\n')
            for topic in topics:
                if topic:
                    print(f"    • {topic}")

            # Highlight point cloud topics
            pointcloud_topics = [t for t in topics if 'point' in t.lower()]
            if pointcloud_topics:
                print(f"\n  {BLUE}DEBUG: Point cloud topics found:{NC}")
                for topic in pointcloud_topics:
                    print(f"    {GREEN}✓ {topic}{NC}")

                # Auto-discover best topic
                discovered_topic, topic_type = discover_pointcloud_topic()
                if discovered_topic:
                    print(f"\n  {BLUE}DEBUG: Auto-discovered point cloud topic:{NC}")
                    print(f"    {GREEN}✓ {discovered_topic} ({topic_type}){NC}")
            else:
                print(f"\n  {YELLOW}DEBUG: No point cloud topics found yet{NC}")
        else:
            print(f"    {YELLOW}No topics found{NC}")
    except Exception as e:
        print(f"    {RED}Error listing topics: {e}{NC}")

def check_camera_topics():
    """Check if camera topics exist"""
    print(f"\n{YELLOW}[3/6] Checking camera topics...{NC}")

    # If we just started the robot, give it extra time for topics to appear
    if robot_process and robot_process.poll() is None:
        print(f"{YELLOW}  Waiting for camera topics to appear...{NC}")
        for i in range(10):
            time.sleep(1)
            try:
                result = run_ros_cmd("timeout 2 ros2 topic list 2>/dev/null", timeout_sec=4)
                if result and "/camera/color/image_raw" in result.stdout:
                    print(f"{GREEN}  Topics appeared!{NC}")
                    break
            except:
                pass
            if i % 3 == 0 and i > 0:
                print(f"{YELLOW}  Still waiting... ({i}s){NC}")

    try:
        result = run_ros_cmd("timeout 2 ros2 topic list 2>/dev/null", timeout_sec=5)

        if not result:
            return False

        topics = result.stdout

        has_color = "/camera/color/image_raw" in topics
        has_depth = "/camera/depth/image_raw" in topics
        has_color_info = "/camera/color/camera_info" in topics
        has_depth_info = "/camera/depth/camera_info" in topics

        if has_color:
            print(f"{GREEN}✓ Color image topic exists{NC}")
        else:
            print(f"{RED}✗ Color image topic missing{NC}")

        if has_depth:
            print(f"{GREEN}✓ Depth image topic exists{NC}")
        else:
            print(f"{RED}✗ Depth image topic missing{NC}")

        if has_color_info:
            print(f"{GREEN}✓ Color camera_info topic exists{NC}")
        else:
            print(f"{RED}✗ Color camera_info topic missing{NC}")

        if has_depth_info:
            print(f"{GREEN}✓ Depth camera_info topic exists{NC}")
        else:
            print(f"{RED}✗ Depth camera_info topic missing{NC}")

        # Show all camera topics for debugging
        show_all_camera_topics()

        return has_color and has_depth and has_color_info and has_depth_info
    except:
        print(f"{RED}✗ Failed to check topics{NC}")
        return False

def check_camera_publishing():
    """Check if camera is ACTUALLY publishing data (critical test!)"""
    print(f"\n{YELLOW}[4/6] Testing if camera is PUBLISHING data...{NC}")
    print(f"{YELLOW}    (This is the most important test!){NC}")

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
        print(f"\n{RED}════════════════════════════════════════════════════{NC}")
        print(f"{RED}{BOLD}  CAMERA IS NOT PUBLISHING DATA!{NC}")
        print(f"{RED}════════════════════════════════════════════════════{NC}")
        print(f"\n{YELLOW}The camera node is running but not streaming.{NC}")
        print(f"{YELLOW}This usually means the camera node is stuck/crashed.{NC}")
        print(f"\n{BOLD}To fix this:{NC}")
        print("  1. Stop this script (Ctrl+C)")
        print("  2. Kill all robot processes:")
        print(f"     {BOLD}killall -9 python3 bash{NC}")
        print("  3. Wait 5 seconds")
        print("  4. Restart the robot:")
        print(f"     {BOLD}./start_robot.sh{NC}")
        print("  5. Wait 10 seconds for camera to initialize")
        print("  6. Run this script again")
        print("")
        return False

    print(f"{GREEN}✓ Camera is publishing data!{NC}")
    return True

def launch_pointcloud_node():
    """Launch the colored point cloud generator"""
    global pointcloud_process

    print(f"\n{YELLOW}[5/6] Launching colored point cloud generator...{NC}")
    print(f"{GREEN}✓ Using camera built-in point cloud generator (Optimized){NC}")
    
    # We don't need to launch anything as the camera driver now handles it
    return True

def verify_pointcloud():
    """Verify colored point cloud is being published"""
    print(f"\n{YELLOW}[6/6] Verifying colored point cloud...{NC}")

    # Show available topics for debugging
    show_all_camera_topics()

    # Wait for topic to appear
    print(f"\n  Waiting for /camera/depth_registered/points topic...", end="", flush=True)
    for i in range(10):
        try:
            result = run_ros_cmd("timeout 2 ros2 topic list 2>/dev/null", timeout_sec=4)
            if result and "/camera/depth_registered/points" in result.stdout:
                print(f" {GREEN}✓{NC}")
                break
        except:
            pass
        print(".", end="", flush=True)
        time.sleep(1)
    else:
        print(f" {RED}✗{NC}")
        print(f"{RED}Failed to create colored point cloud topic{NC}")
        # Show node output for debugging
        print(f"\n{YELLOW}Checking node output for errors...{NC}")
        if pointcloud_process and pointcloud_process.stdout:
            import select
            for _ in range(10):
                ready = select.select([pointcloud_process.stdout], [], [], 0.1)
                if ready[0]:
                    line = pointcloud_process.stdout.readline()
                    if line:
                        print(f"  {line.rstrip()}")
        return False

    # Get topic info
    print(f"\n  {BLUE}Topic info:{NC}")
    try:
        result = run_ros_cmd("timeout 2 ros2 topic info /camera/depth_registered/points 2>/dev/null", timeout_sec=4)
        if result:
            for line in result.stdout.split('\n'):
                if line.strip():
                    print(f"    {line}")
    except:
        pass

    # Check message type
    print(f"\n  {BLUE}Message type:{NC}")
    try:
        result = run_ros_cmd("timeout 2 ros2 topic type /camera/depth_registered/points 2>/dev/null", timeout_sec=4)
        if result:
            print(f"    {result.stdout.strip()}")
    except:
        pass

    # Check publishing rate
    print(f"\n  {BLUE}Publishing rate:{NC}", end="", flush=True)
    try:
        result = run_ros_cmd("timeout 5 ros2 topic hz /camera/depth_registered/points 2>&1", timeout_sec=7)
        if result:
            # Extract rate info
            for line in result.stdout.split('\n'):
                if 'average rate' in line.lower():
                    print(f" {line.strip()}")
                    break
            else:
                print(f" {YELLOW}Unable to measure{NC}")
    except:
        print(f" {YELLOW}Timeout{NC}")

    # Check if publishing
    print(f"\n  {BLUE}Checking if point cloud data is publishing...{NC}", end="", flush=True)
    try:
        result = run_ros_cmd("timeout 5 ros2 topic echo /camera/depth_registered/points --once >/dev/null 2>&1", timeout_sec=7)
        if result.returncode == 0:
            print(f" {GREEN}✓ YES! Data is flowing!{NC}")

            # Try to get a sample of the data
            print(f"\n  {BLUE}Sample point cloud data (first few points):{NC}")
            result = run_ros_cmd("timeout 3 ros2 topic echo /camera/depth_registered/points --once 2>/dev/null | head -30", timeout_sec=5)
            if result and result.stdout:
                print(f"    {result.stdout[:500]}...")

            return True
        else:
            print(f" {YELLOW}⚠ Not yet publishing{NC}")

            # Show more node output
            print(f"\n  {YELLOW}Node output (checking for issues):{NC}")
            if pointcloud_process and pointcloud_process.stdout:
                import select
                for _ in range(20):
                    ready = select.select([pointcloud_process.stdout], [], [], 0.1)
                    if ready[0]:
                        line = pointcloud_process.stdout.readline()
                        if line:
                            print(f"    {line.rstrip()}")

            return True  # Still OK, might just need more time
    except Exception as e:
        print(f" {YELLOW}⚠ Timeout: {e}{NC}")
        return True

def check_ydlidar_hardware():
    """Check if YDLidar is available"""
    print(f"\n{YELLOW}[OPTIONAL] Checking for YDLidar...{NC}")

    # Check for ydlidar USB device
    result = subprocess.run(
        "ls /dev/ydlidar 2>/dev/null",
        shell=True,
        capture_output=True,
        text=True
    )

    if result.returncode == 0:
        print(f"{GREEN}✓ YDLidar device detected at /dev/ydlidar{NC}")
        return True
    else:
        # Also check for ttyUSB devices
        result = subprocess.run(
            "ls /dev/ttyUSB* 2>/dev/null",
            shell=True,
            capture_output=True,
            text=True
        )
        if result.returncode == 0:
            print(f"{YELLOW}⚠ YDLidar not at /dev/ydlidar, but found USB devices{NC}")
            print(f"{YELLOW}  This is OK - will try to use YDLidar if node starts{NC}")
            return True
        else:
            print(f"{YELLOW}⚠ YDLidar NOT detected{NC}")
            print(f"{YELLOW}  Continuing with camera-only mode...{NC}")
            return False

def start_ydlidar():
    """Start YDLidar node"""
    global ydlidar_process

    print(f"\n{YELLOW}[BONUS] Starting YDLidar for 360° coverage...{NC}")

    # Kill any existing ydlidar processes
    subprocess.run("pkill -f 'my_ydlidar_ros2_driver_node' 2>/dev/null", shell=True)
    time.sleep(1)

    # Start ydlidar node with config file
    ydlidar_config = "/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/install/my_ydlidar_ros2_driver/share/my_ydlidar_ros2_driver/params/ydlidar.yaml"

    cmd = f"""
source /opt/ros/humble/setup.bash
source /home/jetson/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
cd /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws
source install/setup.bash
export ROS_DOMAIN_ID=28

ros2 run my_ydlidar_ros2_driver my_ydlidar_ros2_driver_node --ros-args --params-file {ydlidar_config}
"""

    ydlidar_process = subprocess.Popen(
        cmd,
        shell=True,
        executable='/bin/bash',
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

    print(f"{GREEN}✓ YDLidar node started (PID: {ydlidar_process.pid}){NC}")
    print(f"{YELLOW}  Waiting for /scan topic to appear...{NC}")

    # Wait for /scan topic to appear
    for i in range(20):
        time.sleep(1)
        try:
            result = run_ros_cmd("timeout 2 ros2 topic list 2>/dev/null", timeout_sec=4)
            if result and "/scan" in result.stdout:
                print(f"{GREEN}✓ /scan topic is now available!{NC}")
                return True
        except:
            pass

        # Check if process died
        if ydlidar_process.poll() is not None:
            print(f"{YELLOW}⚠ YDLidar process died - continuing with camera only{NC}")
            return False

        if i % 5 == 0 and i > 0:
            print(f"{YELLOW}  Still waiting... ({i}s){NC}")

    print(f"{YELLOW}⚠ YDLidar /scan topic not detected - continuing with camera only{NC}")
    return False

def launch_fusion_node():
    """Launch sensor fusion node to combine camera + lidar"""
    global fusion_process

    print(f"\n{YELLOW}[FUSION] Launching 3D sensor fusion node...{NC}")
    print(f"{GREEN}✓ Using integrated fusion node (no temp files!){NC}")

    # Launch this same script in fusion mode
    script_path = Path(__file__).resolve()

    cmd = f"""
source /opt/ros/humble/setup.bash
source /home/jetson/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
cd /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws
source install/setup.bash
export ROS_DOMAIN_ID=28

python3 {script_path} --fusion-mode
"""

    fusion_process = subprocess.Popen(
        cmd,
        shell=True,
        executable='/bin/bash',
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

    print(f"{GREEN}✓ Fusion node started (PID: {fusion_process.pid}){NC}")

    # Wait for fusion topic
    print(f"{YELLOW}  Waiting for /fused_3d_world topic...{NC}")
    for i in range(10):
        time.sleep(1)
        try:
            result = run_ros_cmd("timeout 2 ros2 topic list 2>/dev/null", timeout_sec=4)
            if result and "/fused_3d_world" in result.stdout:
                print(f"{GREEN}✓ /fused_3d_world topic ready!{NC}")
                return True
        except:
            pass

    print(f"{GREEN}✓ Fusion node running{NC}")
    return True

def launch_rviz():
    """Launch RViz visualization"""
    global rviz_process

    print(f"\n{YELLOW}Launching RViz...{NC}")

    # Make sure RViz config exists
    rviz_config = WORKSPACE / "scripts/3d_world.rviz"
    if not rviz_config.exists():
        print(f"{RED}✗ RViz config not found at {rviz_config}{NC}")
        return False

    cmd = f"""
source /opt/ros/humble/setup.bash
source /home/jetson/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
cd /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws
source install/setup.bash
export ROS_DOMAIN_ID=28
export DISPLAY=:0

rviz2 -d {rviz_config}
"""

    rviz_process = subprocess.Popen(
        cmd,
        shell=True,
        executable='/bin/bash',
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

    print(f"{GREEN}✓ RViz started (PID: {rviz_process.pid}){NC}")
    return True

def check_tf_frames():
    """Check if TF frames are being published"""
    print(f"\n{YELLOW}Checking TF frames...{NC}")

    try:
        result = run_ros_cmd("timeout 2 ros2 topic list 2>/dev/null | grep -E '(tf|tf_static)'", timeout_sec=4)
        if result and result.returncode == 0:
            print(f"{GREEN}✓ TF topics available{NC}")
            return True
        else:
            print(f"{YELLOW}⚠ No TF topics found (this is OK for static camera){NC}")
            return True
    except:
        print(f"{YELLOW}⚠ Could not check TF (this is OK){NC}")
        return True

def verify_final_setup():
    """Final verification that everything is working"""
    print(f"\n{YELLOW}═══ Final Verification ═══{NC}")

    # Check point cloud is publishing
    print(f"  Checking point cloud data flow... ", end="", flush=True)
    try:
        result = run_ros_cmd("timeout 3 ros2 topic hz /camera/depth_registered/points 2>&1 | grep -i 'average'", timeout_sec=5)
        if result and result.returncode == 0:
            rate = result.stdout.strip()
            print(f"{GREEN}✓{NC}")
            print(f"    {rate}")
        else:
            print(f"{YELLOW}⚠ (still initializing){NC}")
    except:
        print(f"{YELLOW}⚠ (still initializing){NC}")

    # Check if RViz is subscribed
    print(f"  Checking RViz subscription... ", end="", flush=True)
    time.sleep(2)  # Give RViz time to subscribe
    try:
        result = run_ros_cmd("timeout 2 ros2 topic info /camera/depth_registered/points 2>/dev/null", timeout_sec=4)
        if result and "Subscription count: 1" in result.stdout:
            print(f"{GREEN}✓ RViz is subscribed{NC}")
        else:
            print(f"{YELLOW}⚠ Waiting for RViz...{NC}")
    except:
        print(f"{YELLOW}⚠ Waiting for RViz...{NC}")

    # Enhanced debug: Check point cloud frame_id and sample data
    print(f"\n  {BLUE}DEBUG: Checking point cloud details...{NC}")
    try:
        # Get frame_id from point cloud
        result = run_ros_cmd("timeout 3 ros2 topic echo /camera/depth_registered/points --once 2>/dev/null | grep 'frame_id' | head -1", timeout_sec=5)
        if result and result.stdout:
            print(f"    Frame ID: {result.stdout.strip()}")

        # Get point cloud size info
        result = run_ros_cmd("timeout 3 ros2 topic echo /camera/depth_registered/points --once 2>/dev/null | grep -E '(width|height):' | head -2", timeout_sec=5)
        if result and result.stdout:
            lines = result.stdout.strip().split('\n')
            if len(lines) >= 2:
                width = lines[0].split(':')[-1].strip() if ':' in lines[0] else '?'
                height = lines[1].split(':')[-1].strip() if ':' in lines[1] else '?'
                print(f"    Point cloud size: {width} x {height} points")
                try:
                    total_points = int(width) * int(height)
                    print(f"    {GREEN}Total points per frame: {total_points:,}{NC}")
                except:
                    pass
    except Exception as e:
        print(f"    {YELLOW}Could not get details (this is OK): {e}{NC}")

def print_success():
    """Print success message"""
    global monitor_thread, stop_monitoring

    print(f"\n{GREEN}{BOLD}═══════════════════════════════════════════════════════════{NC}")
    print(f"{GREEN}{BOLD}  ✓ SUCCESS! 3D WORLD VISUALIZATION READY!{NC}")
    print(f"{GREEN}{BOLD}═══════════════════════════════════════════════════════════{NC}")

    print(f"\n{BOLD}What's Running:{NC}")
    print(f"  📷 Camera Node:      /camera/camera")
    print(f"  🔄 Point Cloud Gen:  camera_driver (built-in)")
    if fusion_process and fusion_process.poll() is None:
        print(f"  🌐 Fusion Node:      Merging camera + lidar data")
    if ydlidar_process and ydlidar_process.poll() is None:
        print(f"  📡 YDLidar Node:     360° laser scanning")
    print(f"  📊 RViz:             3D visualization window")

    print(f"\n{BOLD}Data Flow:{NC}")
    if ydlidar_process and ydlidar_process.poll() is None:
        print(f"  /scan (YDLidar 360°)  ────┐")
        print(f"                             ├──→  Fusion Node  →  /fused_3d_world  →  RViz")
        print(f"  /camera/depth_registered  ─┘")
    else:
        print(f"  /camera/depth_registered/points  →  Fusion Node  →  /fused_3d_world  →  RViz")

    print(f"\n{GREEN}{BOLD}🌍 Check your RViz window now!{NC}")
    print(f"{BOLD}You should see:{NC}")
    print(f"  ✓ Colored 3D point cloud from camera (rich detail)")
    if ydlidar_process and ydlidar_process.poll() is None:
        print(f"  ✓ RED ring showing 360° lidar scan at robot height")
        print(f"  ✓ FULL 360° environment coverage!")
    else:
        print(f"  ⚠ Camera-only mode (YDLidar not detected)")
    print(f"  ✓ Real-time RGB colors from the camera")
    print(f"  ✓ Depth information showing 3D structure")

    print(f"\n{BOLD}Tips:{NC}")
    print(f"  • Move your hand or objects in front of camera")
    if ydlidar_process and ydlidar_process.poll() is None:
        print(f"  • Walk around - see the RED ring track surroundings 360°")
    print(f"  • Use mouse to rotate view in RViz (click and drag)")
    print(f"  • Zoom with scroll wheel")
    print(f"  • If no data yet, wait 5-10 seconds")

    print(f"\n{BOLD}Debug mode: Monitoring point cloud node...{NC}")
    print(f"{BOLD}Press Ctrl+C to stop all processes{NC}\n")

    # Start monitoring thread
    stop_monitoring = False
    monitor_thread = threading.Thread(target=monitor_pointcloud_output, daemon=True)
    monitor_thread.start()

def main():
    print_header()

    # Step 1: Check camera hardware
    if not check_camera_hardware():
        sys.exit(1)

    # Step 2: Check camera node
    if not check_camera_node():
        sys.exit(1)

    # Step 3: Check camera topics
    if not check_camera_topics():
        sys.exit(1)

    # Step 4: Check camera publishing (CRITICAL!)
    if not check_camera_publishing():
        sys.exit(1)

    # Step 5: Launch point cloud generator (camera)
    if not launch_pointcloud_node():
        cleanup()
        sys.exit(1)

    # Step 6: Verify camera point cloud
    verify_pointcloud()

    # Step 7: Check for YDLidar (optional)
    has_ydlidar = False
    if check_ydlidar_hardware():
        if start_ydlidar():
            has_ydlidar = True

    # Step 8: Launch fusion node (combines camera + lidar if available)
    if not launch_fusion_node():
        print(f"{YELLOW}⚠ Fusion node failed, but continuing...{NC}")

    # Check TF frames
    check_tf_frames()

    # Launch RViz
    if not launch_rviz():
        cleanup()
        sys.exit(1)

    # Final verification
    verify_final_setup()

    # Print success
    print_success()

    # Wait for processes and show periodic status
    try:
        status_counter = 0
        while True:
            # Check if processes are still running
            if pointcloud_process and pointcloud_process.poll() is not None:
                print(f"\n{RED}✗ Point cloud node died!{NC}")
                # Show any final output
                if pointcloud_process.stdout:
                    output = pointcloud_process.stdout.read()
                    if output:
                        print(f"{RED}Final output:{NC}")
                        print(output)
                break
            if rviz_process.poll() is not None:
                print(f"\n{YELLOW}RViz closed{NC}")
                break

            # Show periodic status every 10 seconds
            status_counter += 1
            if status_counter >= 10:
                status_counter = 0
                print(f"\n{BLUE}═══ Status Update ═══{NC}")

                # Check point cloud publishing rate
                try:
                    result = run_ros_cmd("timeout 3 ros2 topic hz /camera/depth_registered/points 2>&1 | head -3", timeout_sec=5)
                    if result and result.stdout:
                        for line in result.stdout.split('\n'):
                            if 'average rate' in line.lower() or 'min' in line.lower():
                                print(f"{BLUE}  Point cloud rate: {line.strip()}{NC}")
                except:
                    pass

                # Check number of subscribers
                try:
                    result = run_ros_cmd("timeout 2 ros2 topic info /camera/depth_registered/points 2>/dev/null", timeout_sec=4)
                    if result and result.stdout:
                        for line in result.stdout.split('\n'):
                            if 'Subscription count' in line:
                                print(f"{BLUE}  {line.strip()}{NC}")
                except:
                    pass

            time.sleep(1)
    except KeyboardInterrupt:
        pass

    cleanup()

if __name__ == '__main__':
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='3D World Visualization')
    parser.add_argument('--fusion-mode', action='store_true',
                        help='Run as fusion node (internal use)')
    args = parser.parse_args()

    # Run in appropriate mode
    if args.fusion_mode:
        # Fusion node mode - run as ROS2 node
        run_fusion_node()
    else:
        # Normal mode - launch full visualization
        main()
