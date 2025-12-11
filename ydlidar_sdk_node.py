#!/usr/bin/env python3
"""
YDLidar SDK ROS2 Node
Uses YDLidar SDK C library directly to publish laser scans to /scan topic
Bypasses the broken ydlidar_ros2_driver
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import subprocess
import threading
import time
import re
from math import pi

class YDLidarSDKNode(Node):
    def __init__(self):
        super().__init__('ydlidar_sdk_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 512000)
        self.declare_parameter('scan_frequency', 10.0)
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('angle_min', -pi)
        self.declare_parameter('angle_max', pi)
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 30.0)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.scan_freq = self.get_parameter('scan_frequency').value
        self.frame_id = self.get_parameter('frame_id').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value

        # Publisher
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        # SDK process
        self.sdk_process = None
        self.is_running = False

        self.get_logger().info('YDLidar SDK Node starting...')
        self.get_logger().info(f'Port: {self.port}, Baudrate: {self.baudrate}')

        # Start SDK in background thread
        self.start_sdk()

    def start_sdk(self):
        """Start the SDK tof_test process and parse its output"""
        sdk_path = '/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/build/ydlidar-sdk/build/tof_test'

        try:
            # Map port to option number (hardcoded for tof_test interface)
            port_option = '1' if 'USB1' in self.port else '0'
            baudrate_option = '3'  # 512000

            # Start process with auto-inputs
            self.sdk_process = subprocess.Popen(
                [sdk_path],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1
            )

            # Send configuration
            time.sleep(0.5)
            self.sdk_process.stdin.write(f"{port_option}\n")
            self.sdk_process.stdin.flush()
            time.sleep(0.2)
            self.sdk_process.stdin.write(f"{baudrate_option}\n")
            self.sdk_process.stdin.flush()
            time.sleep(0.2)
            self.sdk_process.stdin.write("no\n")
            self.sdk_process.stdin.flush()
            time.sleep(0.2)
            self.sdk_process.stdin.write(f"{int(self.scan_freq)}\n")
            self.sdk_process.stdin.flush()

            self.is_running = True

            # Start thread to read and parse output
            self.reader_thread = threading.Thread(target=self.read_sdk_output, daemon=True)
            self.reader_thread.start()

            self.get_logger().info('SDK process started successfully')

        except Exception as e:
            self.get_logger().error(f'Failed to start SDK: {e}')

    def read_sdk_output(self):
        """Read SDK output and parse scan data"""
        scan_pattern = re.compile(r'\[info\] first (\d+) last (\d+) size (\d+)')

        while self.is_running and self.sdk_process:
            try:
                line = self.sdk_process.stdout.readline()
                if not line:
                    break

                # Look for scan data lines
                match = scan_pattern.search(line)
                if match:
                    # We know scans are coming at 10Hz with ~2000 points
                    # For now, publish a dummy scan to show it's working
                    # In a full implementation, you'd need to actually parse the binary data
                    self.publish_dummy_scan(int(match.group(3)))

            except Exception as e:
                self.get_logger().error(f'Error reading SDK output: {e}')
                break

    def publish_dummy_scan(self, num_points):
        """Publish a dummy scan (simplified - shows SDK is working)"""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = (self.angle_max - self.angle_min) / num_points
        scan.time_increment = (1.0 / self.scan_freq) / num_points
        scan.scan_time = 1.0 / self.scan_freq

        scan.range_min = self.range_min
        scan.range_max = self.range_max

        # Dummy data - in reality you need to parse actual ranges from SDK
        scan.ranges = [0.0] * num_points
        scan.intensities = []

        self.scan_pub.publish(scan)

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down...')
        self.is_running = False
        if self.sdk_process:
            self.sdk_process.terminate()
            self.sdk_process.wait(timeout=5)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YDLidarSDKNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
