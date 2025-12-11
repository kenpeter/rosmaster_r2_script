#!/usr/bin/env python3
"""
YDLidar TG30 ROS2 Test Script
Tests the fixed YDLidar driver and saves scan data for visualization
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
import time
import json
import numpy as np
import subprocess
import sys
import os

class LidarTester(Node):
    def __init__(self):
        super().__init__('lidar_ros2_tester')

        # Use SensorDataQoS to match the driver (BEST_EFFORT reliability)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            sensor_qos)
        self.scan_count = 0
        self.scan_data = []
        self.latest_scan = None
        self.start_time = time.time()

    def scan_callback(self, msg):
        self.scan_count += 1
        self.latest_scan = msg

        # Convert ranges to list and filter out invalid values
        ranges = np.array(msg.ranges)
        valid_mask = (ranges > msg.range_min) & (ranges < msg.range_max)
        valid_ranges = ranges[valid_mask]

        scan_info = {
            'scan_number': self.scan_count,
            'timestamp': time.time(),
            'frame_id': msg.header.frame_id,
            'angle_min': float(msg.angle_min),
            'angle_max': float(msg.angle_max),
            'angle_increment': float(msg.angle_increment),
            'range_min': float(msg.range_min),
            'range_max': float(msg.range_max),
            'scan_time': float(msg.scan_time),
            'total_points': len(msg.ranges),
            'valid_points': len(valid_ranges),
            'min_distance': float(np.min(valid_ranges)) if len(valid_ranges) > 0 else None,
            'max_distance': float(np.max(valid_ranges)) if len(valid_ranges) > 0 else None,
            'avg_distance': float(np.mean(valid_ranges)) if len(valid_ranges) > 0 else None,
        }

        self.scan_data.append(scan_info)

        # Print progress every 5 scans
        if self.scan_count % 5 == 0:
            print(f"  📊 Scan #{self.scan_count}: {scan_info['valid_points']:4d} points, "
                  f"avg: {scan_info['avg_distance']:.2f}m, "
                  f"rate: {self.scan_count/(time.time()-self.start_time):.1f}Hz")

def save_scan_to_csv(scan_msg, filename):
    """Save scan data to CSV for easy analysis"""
    with open(filename, 'w') as f:
        f.write("angle_deg,range_m,intensity\n")
        for i, (r, intensity) in enumerate(zip(scan_msg.ranges, scan_msg.intensities)):
            if r > scan_msg.range_min and r < scan_msg.range_max:
                angle_deg = np.degrees(scan_msg.angle_min + i * scan_msg.angle_increment)
                f.write(f"{angle_deg:.2f},{r:.3f},{intensity}\n")
    print(f"  💾 CSV saved: {filename}")

def save_scan_for_rviz(scan_msg, output_dir):
    """Save scan in ROS bag format for RViz visualization"""
    import sqlite3
    from rclpy.serialization import serialize_message

    bag_path = os.path.join(output_dir, 'lidar_scan')

    # Create bag using ros2 bag API would be complex, so we'll create a simple launch file instead
    rviz_config = os.path.join(output_dir, 'lidar_view.rviz')
    with open(rviz_config, 'w') as f:
        f.write("""Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
    - Class: rviz_default_plugins/LaserScan
      Name: LaserScan
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /scan
      Enabled: true
      Size (m): 0.05
      Color: 255; 0; 0
      Alpha: 1.0
      Decay Time: 0
  Global Options:
    Fixed Frame: laser
  Views:
    Current:
      Class: rviz_default_plugins/TopDownOrtho
      Name: TopDownOrtho
      Angle: 0
      Scale: 50
""")
    print(f"  🎨 RViz config saved: {rviz_config}")
    return rviz_config

def main():
    print("=" * 70)
    print("  YDLIDAR TG30 ROS2 TEST")
    print("=" * 70)
    print("\n🚀 Testing YDLidar TG30 with permanent udev rule setup")
    print("   Device: /dev/ydlidar -> ttyUSB1 (Silicon Labs CP2102)")
    print("   Model: TG30 (Model Code 101)")
    print("   Config: 512000 baud, 20K sample rate, 10Hz\n")

    # Check if udev rule exists
    if os.path.exists('/etc/udev/rules.d/99-ydlidar.rules'):
        print("✅ Permanent udev rule found")
    else:
        print("⚠️  Warning: udev rule not found at /etc/udev/rules.d/99-ydlidar.rules")

    # Check if device exists
    if os.path.exists('/dev/ydlidar'):
        device_link = os.readlink('/dev/ydlidar')
        print(f"✅ Device found: /dev/ydlidar -> {device_link}\n")
    else:
        print("❌ Error: /dev/ydlidar not found!")
        print("   Run: sudo udevadm control --reload-rules && sudo udevadm trigger\n")
        return

    print("📋 Test Plan:")
    print("   1. Launch YDLidar driver in background")
    print("   2. Collect 30 scans (~3 seconds)")
    print("   3. Save scan data to files")
    print("   4. Generate RViz config for visualization\n")

    input("Press ENTER to start the test...")

    # Start the driver
    print("\n🔄 Starting YDLidar driver...")
    driver_proc = subprocess.Popen(
        ['ros2', 'launch', 'my_ydlidar_ros2_driver', 'ydlidar_launch.py'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )

    # Wait for driver to initialize
    print("   Waiting for driver to initialize (5 seconds)...")
    time.sleep(5)

    # Initialize ROS2
    rclpy.init()
    tester = LidarTester()

    print("\n📡 Collecting scan data...\n")

    # Collect scans
    max_scans = 30
    timeout = 10
    start_time = time.time()

    try:
        while rclpy.ok() and tester.scan_count < max_scans:
            rclpy.spin_once(tester, timeout_sec=0.1)

            if time.time() - start_time > timeout:
                print(f"\n⚠️  Timeout! Only collected {tester.scan_count} scans")
                break

    except KeyboardInterrupt:
        print("\n\n🛑 Interrupted by user")

    # Stop the driver
    print("\n\n🛑 Stopping driver...")
    driver_proc.terminate()
    driver_proc.wait(timeout=5)

    # Calculate statistics
    elapsed = time.time() - tester.start_time

    print("\n" + "=" * 70)
    print("  TEST RESULTS")
    print("=" * 70)

    if tester.scan_count == 0:
        print("\n❌ No scan data received!")
        print("\nTroubleshooting:")
        print("  - Check if driver is running: ros2 node list")
        print("  - Check topics: ros2 topic list")
        print("  - Check device: ls -l /dev/ydlidar")
        tester.destroy_node()
        rclpy.shutdown()
        return

    print(f"\n✅ Successfully collected {tester.scan_count} scans in {elapsed:.2f} seconds")
    print(f"   Scan rate: {tester.scan_count/elapsed:.2f} Hz")

    # Analyze the data
    valid_points_list = [s['valid_points'] for s in tester.scan_data]
    avg_distances = [s['avg_distance'] for s in tester.scan_data if s['avg_distance'] is not None]

    print(f"\n📊 Scan Statistics:")
    print(f"   Average points per scan: {np.mean(valid_points_list):.0f}")
    print(f"   Min points per scan: {np.min(valid_points_list)}")
    print(f"   Max points per scan: {np.max(valid_points_list)}")

    if avg_distances:
        print(f"\n📏 Distance Statistics:")
        print(f"   Average distance: {np.mean(avg_distances):.2f} m")
        print(f"   Min distance seen: {np.min(avg_distances):.2f} m")
        print(f"   Max distance seen: {np.max(avg_distances):.2f} m")

    # Save files
    output_dir = '/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/scripts'

    # Save JSON summary
    summary_file = os.path.join(output_dir, 'lidar_test_results.json')
    summary = {
        'test_date': time.strftime('%Y-%m-%d %H:%M:%S'),
        'device': '/dev/ydlidar',
        'model': 'TG30',
        'total_scans': tester.scan_count,
        'duration_seconds': elapsed,
        'scan_rate_hz': tester.scan_count/elapsed,
        'statistics': {
            'avg_points_per_scan': float(np.mean(valid_points_list)),
            'min_points': int(np.min(valid_points_list)),
            'max_points': int(np.max(valid_points_list)),
        },
        'scans': tester.scan_data
    }

    if avg_distances:
        summary['statistics']['avg_distance_m'] = float(np.mean(avg_distances))
        summary['statistics']['min_distance_m'] = float(np.min(avg_distances))
        summary['statistics']['max_distance_m'] = float(np.max(avg_distances))

    with open(summary_file, 'w') as f:
        json.dump(summary, f, indent=2)
    print(f"\n💾 Summary saved: {summary_file}")

    # Save latest scan to CSV
    if tester.latest_scan:
        csv_file = os.path.join(output_dir, 'lidar_scan.csv')
        save_scan_to_csv(tester.latest_scan, csv_file)

        # Create RViz config
        rviz_config = save_scan_for_rviz(tester.latest_scan, output_dir)

    # Show sample scan
    if tester.scan_data:
        print(f"\n📋 Sample scan (scan #{tester.scan_count}):")
        sample = tester.scan_data[-1]
        print(f"   Frame ID: {sample['frame_id']}")
        print(f"   Angle range: {np.degrees(sample['angle_min']):.1f}° to {np.degrees(sample['angle_max']):.1f}°")
        print(f"   Distance range: {sample['range_min']:.2f} to {sample['range_max']:.2f} m")
        print(f"   Valid points: {sample['valid_points']}/{sample['total_points']}")
        print(f"   Scan time: {sample['scan_time']:.3f} seconds")

    print("\n" + "=" * 70)
    print("  VISUALIZATION")
    print("=" * 70)
    print("\n🎨 To view the scan data in RViz2:")
    print("\n   Terminal 1 - Start the driver:")
    print("   cd /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws")
    print("   source install/setup.bash")
    print("   ros2 launch my_ydlidar_ros2_driver ydlidar_launch.py")
    print("\n   Terminal 2 - Start RViz2:")
    print("   cd /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws")
    print("   source install/setup.bash")
    print(f"   rviz2 -d {rviz_config}")
    print("\n   You should see the lidar scan data in real-time!")
    print("\n✅ Test completed successfully!")
    print(f"\n📁 Output files in: {output_dir}")
    print(f"   - lidar_test_results.json  (detailed statistics)")
    print(f"   - lidar_scan.csv           (scan data for analysis)")
    print(f"   - lidar_view.rviz          (RViz2 config)\n")

    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
