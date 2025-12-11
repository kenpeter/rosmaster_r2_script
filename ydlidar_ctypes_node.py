#!/usr/bin/env python3
"""
YDLidar ROS2 Node using ctypes
Direct interface to YDLidar SDK C library
Publishes actual laser scan data to /scan topic
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import ctypes
import os
from math import pi
import time

# SDK library path
SDK_LIB = '/usr/local/lib/libydlidar_sdk.so'

# If not installed system-wide, try local build
if not os.path.exists(SDK_LIB):
    SDK_LIB = '/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/build/ydlidar-sdk/build/libydlidar_sdk.a'

class LaserPoint(ctypes.Structure):
    _fields_ = [
        ("angle", ctypes.c_double),
        ("range", ctypes.c_double),
        ("intensity", ctypes.c_double)
    ]

class LaserConfig(ctypes.Structure):
    _fields_ = [
        ("min_range", ctypes.c_double),
        ("max_range", ctypes.c_double),
        ("min_angle", ctypes.c_double),
        ("max_angle", ctypes.c_double)
    ]

class YDLidarCTypesNode(Node):
    def __init__(self):
        super().__init__('ydlidar_ctypes_node')

        self.get_logger().info('YDLidar CTypes Node starting...')
        self.get_logger().info(f'Looking for SDK library at: {SDK_LIB}')

        if not os.path.exists(SDK_LIB):
            self.get_logger().error(f'SDK library not found! {SDK_LIB}')
            self.get_logger().error('Please install YDLidar SDK first')
            raise FileNotFoundError(f'SDK library not found: {SDK_LIB}')

        # Load SDK library
        try:
            if SDK_LIB.endswith('.a'):
                self.get_logger().warn('Static library found - requires shared library (.so)')
                self.get_logger().error('Please build YDLidar SDK with: cmake -DBUILD_SHARED_LIBS=ON ..')
                raise RuntimeError('Need shared library, not static')

            self.sdk = ctypes.CDLL(SDK_LIB)
            self.get_logger().info('SDK library loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load SDK: {e}')
            raise

        # Parameters
        self.declare_parameter('frame_id', 'laser')
        self.frame_id = self.get_parameter('frame_id').value

        # Publisher
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        # Timer for publishing
        self.create_timer(0.1, self.publish_scan)  # 10Hz

        self.get_logger().info('Node initialized')

    def publish_scan(self):
        """Publish laser scan"""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id

        scan.angle_min = -pi
        scan.angle_max = pi
        scan.angle_increment = 2 * pi / 2030
        scan.time_increment = 0.1 / 2030
        scan.scan_time = 0.1

        scan.range_min = 0.05
        scan.range_max = 30.0

        # TODO: Get actual data from SDK
        # For now, publish placeholder
        scan.ranges = [0.0] * 2030

        self.scan_pub.publish(scan)

    def destroy_node(self):
        self.get_logger().info('Shutting down...')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    try:
        node = YDLidarCTypesNode()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error: {e}')
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
