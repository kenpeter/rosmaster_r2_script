#!/usr/bin/env python3
"""
YDLidar TG30 Test Script
Model: TG30
Model Code: 101
Baudrate: 512000
Firmware: 1.71
Sample Rate: 20kHz
Scan Frequency: 10Hz
Points per Scan: ~1970
"""

import serial
import time
import struct
import sys

class TG30Lidar:
    """TG30 Lidar Controller"""

    # TG30 Specifications
    MODEL_CODE = 101
    BAUDRATE = 512000
    SAMPLE_RATE = 20000  # 20kHz
    SCAN_FREQUENCY = 10  # 10Hz
    EXPECTED_POINTS = 1970

    # Protocol Commands
    CMD_STOP = 0x65
    CMD_SCAN = 0x60
    CMD_FORCE_SCAN = 0x61
    CMD_GET_INFO = 0x90
    CMD_GET_HEALTH = 0x92

    # Packet markers
    PACKET_HEADER = 0xAA55  # Triangle protocol

    def __init__(self, port='/dev/ydlidar'):
        self.port = port
        self.ser = None
        self.running = False

    def connect(self):
        """Connect to TG30 lidar"""
        try:
            print(f"Connecting to TG30 on {self.port} at {self.BAUDRATE} baud...")
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.BAUDRATE,
                timeout=1.0,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )

            # Clear buffers
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            time.sleep(0.5)

            print("✓ Serial connection established")
            return True

        except Exception as e:
            print(f"✗ Failed to connect: {e}")
            return False

    def start_motor(self):
        """Start the lidar motor using DTR"""
        try:
            print("Starting motor (DTR)...")
            self.ser.dtr = True
            time.sleep(2.0)  # Give motor time to spin up to full speed
            print("✓ Motor should be spinning")
            return True
        except Exception as e:
            print(f"✗ Failed to start motor: {e}")
            return False

    def stop_motor(self):
        """Stop the lidar motor"""
        try:
            self.ser.dtr = False
            print("Motor stopped")
        except:
            pass

    def send_command(self, cmd):
        """Send command to lidar"""
        try:
            packet = bytes([0xA5, cmd])
            self.ser.write(packet)
            self.ser.flush()
            return True
        except Exception as e:
            print(f"✗ Failed to send command 0x{cmd:02X}: {e}")
            return False

    def get_device_info(self):
        """Get device information"""
        print("\nGetting device info...")
        self.send_command(self.CMD_GET_INFO)
        time.sleep(0.3)

        if self.ser.in_waiting > 0:
            response = self.ser.read(self.ser.in_waiting)
            print(f"Device info response ({len(response)} bytes): {response.hex()}")

            # Parse response if valid (A5 5A header)
            if len(response) >= 20 and response[0] == 0xA5 and response[1] == 0x5A:
                model = response[6]
                firmware_major = response[7]
                firmware_minor = response[8]
                hardware = response[9]
                serial_num = response[10:18].hex()

                print(f"  Model Code: {model}")
                print(f"  Firmware: {firmware_major}.{firmware_minor}")
                print(f"  Hardware: {hardware}")
                print(f"  Serial: {serial_num}")

                if model == self.MODEL_CODE:
                    print(f"✓ Confirmed TG30 (Model {model})")
                else:
                    print(f"⚠ Model mismatch - expected {self.MODEL_CODE}, got {model}")
                return True
        else:
            print("✗ No device info response")
            return False

    def get_health(self):
        """Get device health status"""
        print("\nChecking health...")
        self.send_command(self.CMD_GET_HEALTH)
        time.sleep(0.3)

        if self.ser.in_waiting > 0:
            response = self.ser.read(self.ser.in_waiting)
            print(f"Health response: {response.hex()}")
            return True
        else:
            print("No health response")
            return False

    def start_scan(self):
        """Start scanning"""
        print("\nStarting scan...")

        # Stop any existing scan
        self.send_command(self.CMD_STOP)
        time.sleep(0.2)
        self.ser.reset_input_buffer()

        # Start scan
        if not self.send_command(self.CMD_SCAN):
            print("Regular scan failed, trying force scan...")
            self.send_command(self.CMD_FORCE_SCAN)

        time.sleep(0.5)

        if self.ser.in_waiting > 0:
            print(f"✓ Scan started - {self.ser.in_waiting} bytes waiting")
            self.running = True
            return True
        else:
            print("✗ No scan data received")
            return False

    def stop_scan(self):
        """Stop scanning"""
        self.running = False
        self.send_command(self.CMD_STOP)
        time.sleep(0.2)
        print("Scan stopped")

    def read_scan_packets(self, duration=10):
        """Read and analyze scan packets"""
        print(f"\nReading scan data for {duration} seconds...")
        print("="*70)

        start_time = time.time()
        packet_buffer = bytearray()

        packet_count = 0
        total_points = 0
        total_bytes = 0

        while time.time() - start_time < duration:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                packet_buffer.extend(data)
                total_bytes += len(data)

            # Parse packets from buffer
            while len(packet_buffer) >= 10:
                # Look for packet header (0xAA 0x55)
                if packet_buffer[0] == 0xAA and packet_buffer[1] == 0x55:
                    sample_quantity = packet_buffer[2]
                    packet_len = 10 + sample_quantity * 2

                    # Check if we have full packet
                    if len(packet_buffer) < packet_len:
                        break

                    try:
                        # Parse packet
                        ct = packet_buffer[3]
                        start_angle_raw = struct.unpack('<H', packet_buffer[4:6])[0]
                        end_angle_raw = struct.unpack('<H', packet_buffer[6:8])[0]

                        start_angle = (start_angle_raw >> 1) / 64.0
                        end_angle = (end_angle_raw >> 1) / 64.0

                        # Read samples
                        samples = []
                        for i in range(sample_quantity):
                            offset = 10 + i * 2
                            dist_raw = struct.unpack('<H', packet_buffer[offset:offset+2])[0]
                            distance = dist_raw / 4.0  # mm
                            samples.append(distance)

                        packet_count += 1
                        total_points += sample_quantity

                        # Display packet info
                        if packet_count <= 10 or packet_count % 10 == 0:
                            avg_dist = sum(s for s in samples if s > 0) / max(1, len([s for s in samples if s > 0]))
                            print(f"Packet {packet_count:4d}: {sample_quantity:3d} points, "
                                  f"angles {start_angle:6.1f}° → {end_angle:6.1f}°, "
                                  f"avg dist {avg_dist:6.1f}mm")

                        # Remove processed packet
                        del packet_buffer[:packet_len]

                    except Exception as e:
                        print(f"Parse error: {e}")
                        del packet_buffer[0]
                else:
                    # No valid header, skip byte
                    del packet_buffer[0]

            time.sleep(0.001)

        # Statistics
        elapsed = time.time() - start_time
        print("="*70)
        print(f"\nStatistics:")
        print(f"  Duration: {elapsed:.2f}s")
        print(f"  Total packets: {packet_count}")
        print(f"  Total points: {total_points}")
        print(f"  Total bytes: {total_bytes}")
        print(f"  Packet rate: {packet_count/elapsed:.1f} packets/sec")
        print(f"  Point rate: {total_points/elapsed:.1f} points/sec")
        print(f"  Data rate: {total_bytes/elapsed:.1f} bytes/sec")
        print(f"  Avg points/packet: {total_points/max(1,packet_count):.1f}")
        print(f"  Estimated scan rate: {packet_count/elapsed:.1f} Hz")

        # Compare with TG30 specs
        print(f"\nTG30 Specifications:")
        print(f"  Expected scan rate: {self.SCAN_FREQUENCY} Hz")
        print(f"  Expected sample rate: {self.SAMPLE_RATE} Hz")
        print(f"  Expected points/scan: ~{self.EXPECTED_POINTS}")

        if packet_count > 0:
            actual_scan_rate = packet_count / elapsed
            actual_point_rate = total_points / elapsed

            if abs(actual_scan_rate - self.SCAN_FREQUENCY) < 2:
                print(f"✓ Scan rate matches TG30 specs")
            else:
                print(f"⚠ Scan rate differs from specs")

            if abs(actual_point_rate - self.SAMPLE_RATE) < 2000:
                print(f"✓ Point rate matches TG30 specs")
            else:
                print(f"⚠ Point rate differs from specs")

    def disconnect(self):
        """Disconnect from lidar"""
        if self.ser and self.ser.is_open:
            self.stop_scan()
            self.stop_motor()
            self.ser.close()
            print("\nDisconnected")


def main():
    print("="*70)
    print("YDLidar TG30 Test Script")
    print("="*70)

    # Get port from command line or use default
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ydlidar'

    # Create lidar instance
    lidar = TG30Lidar(port)

    try:
        # Connect
        if not lidar.connect():
            return 1

        # Start motor
        if not lidar.start_motor():
            return 1

        # Get device info
        lidar.get_device_info()

        # Get health
        lidar.get_health()

        # Start scanning
        if not lidar.start_scan():
            return 1

        # Read scan data
        lidar.read_scan_packets(duration=10)

        print("\n✓ Test completed successfully!")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        lidar.disconnect()

    return 0


if __name__ == '__main__':
    sys.exit(main())
