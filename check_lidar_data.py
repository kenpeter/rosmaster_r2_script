#!/usr/bin/env python3
"""
Quick diagnostic to check what data the lidar is receiving
"""
import subprocess
import sys
import os
import time

# Source ROS2 environment
result = subprocess.run(
    "bash -c 'source /opt/ros/humble/setup.bash && source /home/jetson/yahboomcar_ros2_ws/software/library_ws/install/setup.bash && env'",
    shell=True,
    capture_output=True,
    text=True
)

env = {}
for line in result.stdout.split('\n'):
    if '=' in line:
        key, value = line.split('=', 1)
        env[key] = value

print("Starting driver...")
driver = subprocess.Popen(
    ['/home/jetson/yahboomcar_ros2_ws/software/library_ws/install/ydlidar_ros2_driver/lib/ydlidar_ros2_driver/ydlidar_ros2_driver_node',
     '--ros-args', '--params-file',
     '/home/jetson/yahboomcar_ros2_ws/software/library_ws/install/ydlidar_ros2_driver/share/ydlidar_ros2_driver/params/ydlidar.yaml'],
    env=env,
    stdout=subprocess.DEVNULL,
    stderr=subprocess.DEVNULL
)

time.sleep(6)

print("\nAnalyzing scan data...\n")

# Get scan data
result = subprocess.run(
    ['ros2', 'topic', 'echo', '/scan', '--once'],
    env=env,
    capture_output=True,
    text=True,
    timeout=10
)

# Parse the ranges
lines = result.stdout.split('\n')
in_ranges = False
ranges = []

for line in lines:
    if line.startswith('ranges:'):
        in_ranges = True
        continue
    if in_ranges:
        if line.startswith('- '):
            try:
                val = float(line.strip('- ').strip())
                ranges.append(val)
            except:
                pass
        elif line.startswith('intensities:'):
            break

driver.terminate()
driver.wait()

print(f"Total points in scan: {len(ranges)}")
print(f"Zero/invalid readings: {sum(1 for r in ranges if r == 0.0)}")
print(f"Valid readings: {sum(1 for r in ranges if r > 0.0)}")

if len(ranges) > 0:
    valid = [r for r in ranges if r > 0.0]
    if valid:
        print(f"\nValid range statistics:")
        print(f"  Min distance: {min(valid):.3f}m")
        print(f"  Max distance: {max(valid):.3f}m")
        print(f"  Average: {sum(valid)/len(valid):.3f}m")
        print(f"\nSample of valid readings (first 10):")
        for i, r in enumerate(valid[:10]):
            angle = -180 + (i * 360 / len(ranges))
            print(f"  Angle {angle:6.1f}°: {r:.3f}m")
    else:
        print("\n⚠️  ALL READINGS ARE ZERO!")
        print("\nPossible reasons:")
        print("  1. Lidar is pointing at ceiling/open space")
        print("  2. All objects are outside detection range (0.01m - 50m)")
        print("  3. Lidar lens is obstructed")
        print("  4. Physical setup issue")
        print("\n💡 Try: Point the lidar at a wall or object ~1-2 meters away")
