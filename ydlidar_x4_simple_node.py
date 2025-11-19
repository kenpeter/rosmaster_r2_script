#!/usr/bin/env python3
"""
Simple YDLidar X4 ROS2 node that bypasses the SDK
Directly reads and publishes scan data without health checks
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import struct
import math
import time

class YDLidarX4Node(Node):
    """Direct X4 lidar driver without SDK health checks"""

    # X4 Protocol Constants
    SYNC_BYTE1 = 0xA5
    SYNC_BYTE2 = 0x5A

    # Commands
    CMD_STOP = 0x65
    CMD_SCAN = 0x60
    CMD_GET_INFO = 0x90

    def __init__(self):
        super().__init__('ydlidar_x4_simple')

        # Parameters
        self.declare_parameter('port', '/dev/ydlidar')
        self.declare_parameter('baudrate', 128000)
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('angle_min', -3.14159)  # -180 degrees
        self.declare_parameter('angle_max', 3.14159)   # +180 degrees
        self.declare_parameter('range_min', 0.12)      # 12cm
        self.declare_parameter('range_max', 10.0)      # 10m
        self.declare_parameter('scan_frequency', 8.0)  # 8Hz

        # Get parameters
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value

        # Publisher
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        # Serial connection
        self.serial = None
        try:
            self.get_logger().info(f'Opening {port} at {baudrate} baud...')
            self.serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=1.0,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )

            # Enable DTR to power the motor
            self.serial.setDTR(True)
            time.sleep(0.5)

            self.get_logger().info('Serial port opened, motor powered on')

            # Start scanning
            self.start_scan()

            # Create timer for reading data
            self.create_timer(0.01, self.read_scan_data)  # 100Hz read rate

        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

    def start_scan(self):
        """Send scan command to start scanning"""
        try:
            # Stop any existing scan first
            self.send_command(self.CMD_STOP)
            time.sleep(0.1)

            # Start new scan
            self.send_command(self.CMD_SCAN)
            self.get_logger().info('Scan started')

        except Exception as e:
            self.get_logger().error(f'Failed to start scan: {e}')

    def send_command(self, cmd):
        """Send command to lidar"""
        packet = bytes([self.SYNC_BYTE1, cmd])
        self.serial.write(packet)
        self.serial.flush()

    def read_scan_data(self):
        """Read and publish scan data"""
        try:
            if self.serial.in_waiting < 10:
                return

            # Read packet header
            header = self.serial.read(2)
            if len(header) < 2:
                return

            # Check for valid packet start (0xAA 0x55)
            if header[0] != 0xAA or header[1] != 0x55:
                # Sync: look for start bytes
                return

            # Read packet type (CT) and sample count (LSN)
            ct_lsn = self.serial.read(2)
            if len(ct_lsn) < 2:
                return

            ct = ct_lsn[0]
            lsn = ct_lsn[1]

            # Read FSA (start angle)
            fsa_bytes = self.serial.read(2)
            if len(fsa_bytes) < 2:
                return
            fsa = struct.unpack('<H', fsa_bytes)[0]

            # Read LSA (end angle)
            lsa_bytes = self.serial.read(2)
            if len(lsa_bytes) < 2:
                return
            lsa = struct.unpack('<H', lsa_bytes)[0]

            # Read CS (checksum)
            cs_bytes = self.serial.read(2)
            if len(cs_bytes) < 2:
                return

            # Read sample data (2 bytes per sample)
            sample_count = lsn
            samples_bytes = self.serial.read(sample_count * 2)
            if len(samples_bytes) < sample_count * 2:
                return

            # Parse samples
            samples = []
            for i in range(sample_count):
                distance_raw = struct.unpack('<H', samples_bytes[i*2:i*2+2])[0]
                distance = distance_raw / 4000.0  # Convert to meters
                samples.append(distance)

            # Calculate angles
            start_angle = (fsa >> 1) / 64.0  # FSA format: angle = (FSA >> 1) / 64
            end_angle = (lsa >> 1) / 64.0    # LSA format: angle = (LSA >> 1) / 64

            # Handle angle wraparound
            if end_angle < start_angle:
                end_angle += 360.0

            # Create LaserScan message
            if sample_count > 1:
                angle_increment = math.radians((end_angle - start_angle) / (sample_count - 1))
            else:
                angle_increment = 0.0

            scan_msg = LaserScan()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = self.frame_id

            scan_msg.angle_min = math.radians(start_angle)
            scan_msg.angle_max = math.radians(end_angle)
            scan_msg.angle_increment = angle_increment

            scan_msg.time_increment = 0.0
            scan_msg.scan_time = 0.125  # ~8Hz

            scan_msg.range_min = self.range_min
            scan_msg.range_max = self.range_max

            # Filter and add ranges
            scan_msg.ranges = []
            for distance in samples:
                if distance < self.range_min or distance > self.range_max:
                    scan_msg.ranges.append(float('inf'))
                else:
                    scan_msg.ranges.append(distance)

            scan_msg.intensities = []  # X4 doesn't provide intensity

            # Publish
            self.scan_pub.publish(scan_msg)

        except Exception as e:
            self.get_logger().warn(f'Error reading scan: {e}')

    def destroy_node(self):
        """Cleanup on shutdown"""
        if self.serial:
            try:
                self.send_command(self.CMD_STOP)
                self.serial.setDTR(False)  # Turn off motor
                self.serial.close()
                self.get_logger().info('Lidar stopped and serial closed')
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = YDLidarX4Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
