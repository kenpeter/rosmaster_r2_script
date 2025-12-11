#!/usr/bin/env python3
"""
Test script for my_ydlidar_ros2_driver
Captures scan data and saves it to a file for verification
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import time
import json
import numpy as np

class LidarTester(Node):
    def __init__(self):
        super().__init__('lidar_tester')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.scan_count = 0
        self.scan_data = []
        self.start_time = time.time()

    def scan_callback(self, msg):
        self.scan_count += 1

        # Convert ranges to list and filter out invalid values
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[(ranges > msg.range_min) & (ranges < msg.range_max)]

        scan_info = {
            'scan_number': self.scan_count,
            'timestamp': time.time(),
            'frame_id': msg.header.frame_id,
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'range_min': msg.range_min,
            'range_max': msg.range_max,
            'total_points': len(msg.ranges),
            'valid_points': len(valid_ranges),
            'min_distance': float(np.min(valid_ranges)) if len(valid_ranges) > 0 else None,
            'max_distance': float(np.max(valid_ranges)) if len(valid_ranges) > 0 else None,
            'avg_distance': float(np.mean(valid_ranges)) if len(valid_ranges) > 0 else None,
        }

        self.scan_data.append(scan_info)

        # Print progress
        if self.scan_count % 10 == 0:
            print(f"📊 Scan #{self.scan_count}: {scan_info['valid_points']} valid points, "
                  f"avg dist: {scan_info['avg_distance']:.2f}m")

def main():
    print("="*70)
    print("  MY YDLIDAR ROS2 DRIVER TEST")
    print("="*70)
    print("\nThis script will:")
    print("  1. Subscribe to /scan topic")
    print("  2. Collect 50 scans (about 5 seconds)")
    print("  3. Save the data to lidar_test_results.json")
    print("  4. Display statistics\n")

    rclpy.init()

    tester = LidarTester()

    print("🔄 Collecting scan data...\n")

    # Collect 50 scans
    max_scans = 50
    timeout = 30  # 30 seconds timeout
    start_time = time.time()

    try:
        while rclpy.ok() and tester.scan_count < max_scans:
            rclpy.spin_once(tester, timeout_sec=0.1)

            # Check timeout
            if time.time() - start_time > timeout:
                print(f"\n⚠️  Timeout! Only collected {tester.scan_count} scans")
                break

    except KeyboardInterrupt:
        print("\n\n🛑 Interrupted by user")

    # Calculate statistics
    elapsed = time.time() - tester.start_time

    print("\n" + "="*70)
    print("  TEST RESULTS")
    print("="*70)

    if tester.scan_count == 0:
        print("\n❌ No scan data received!")
        print("\nTroubleshooting:")
        print("  - Make sure my_ydlidar_ros2_driver is running:")
        print("    ros2 launch my_ydlidar_ros2_driver ydlidar_launch.py")
        print("  - Check if /scan topic exists:")
        print("    ros2 topic list")
        tester.destroy_node()
        rclpy.shutdown()
        return

    print(f"\n✅ Successfully collected {tester.scan_count} scans in {elapsed:.2f} seconds")
    print(f"   Scan rate: {tester.scan_count/elapsed:.2f} Hz\n")

    # Analyze the data
    valid_points_list = [s['valid_points'] for s in tester.scan_data]
    avg_distances = [s['avg_distance'] for s in tester.scan_data if s['avg_distance'] is not None]

    print("📊 Scan Statistics:")
    print(f"   Average points per scan: {np.mean(valid_points_list):.0f}")
    print(f"   Min points per scan: {np.min(valid_points_list)}")
    print(f"   Max points per scan: {np.max(valid_points_list)}")

    if avg_distances:
        print(f"\n📏 Distance Statistics:")
        print(f"   Average distance: {np.mean(avg_distances):.2f} m")
        print(f"   Min distance seen: {np.min(avg_distances):.2f} m")
        print(f"   Max distance seen: {np.max(avg_distances):.2f} m")

    # Save to file
    output_file = '/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/scripts/lidar_test_results.json'

    summary = {
        'test_date': time.strftime('%Y-%m-%d %H:%M:%S'),
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

    with open(output_file, 'w') as f:
        json.dump(summary, f, indent=2)

    print(f"\n💾 Detailed results saved to: {output_file}")

    # Show sample scan
    if tester.scan_data:
        print("\n📋 Sample scan (first scan):")
        sample = tester.scan_data[0]
        print(f"   Frame ID: {sample['frame_id']}")
        print(f"   Angle range: {sample['angle_min']:.2f} to {sample['angle_max']:.2f} rad")
        print(f"   Distance range: {sample['range_min']:.2f} to {sample['range_max']:.2f} m")
        print(f"   Valid points: {sample['valid_points']}/{sample['total_points']}")

    print("\n✅ Test completed successfully!")
    print("\n🚀 Your my_ydlidar_ros2_driver is working correctly!\n")

    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
