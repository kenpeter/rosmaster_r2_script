#!/usr/bin/env python3
# Advanced ROS Master Serial Communication Test and Fix

import sys
import time
import serial
import struct

class RosmasterTester:
    def __init__(self, port="/dev/ttyUSB1"):
        self.ser = None
        self.port = port

        self.HEAD = 0xFF
        self.DEVICE_ID = 0xFC
        self.COMPLEMENT = 257 - self.DEVICE_ID

        self.FUNC_AUTO_REPORT = 0x01
        self.FUNC_BEEP = 0x02
        self.FUNC_VERSION = 0x51
        self.FUNC_MOTION = 0x12
        self.FUNC_SET_CAR_TYPE = 0x15
        self.FUNC_REPORT_SPEED = 0x0A
        self.FUNC_REPORT_MPU_RAW = 0x0B

        self.CARTYPE_R2 = 0x05

    def open_serial(self):
        """Open serial port"""
        try:
            self.ser = serial.Serial(self.port, 115200, timeout=1)
            if self.ser.isOpen():
                print(f"✅ Serial port {self.port} opened successfully")
                return True
            else:
                print(f"❌ Failed to open serial port {self.port}")
                return False
        except Exception as e:
            print(f"❌ Error opening serial port: {e}")
            return False

    def close_serial(self):
        """Close serial port"""
        if self.ser and self.ser.isOpen():
            self.ser.close()
            print("Serial port closed")

    def send_command(self, cmd_list):
        """Send command with checksum"""
        cmd = list(cmd_list)  # Make a copy
        checksum = (sum(cmd) + self.COMPLEMENT) & 0xff
        cmd.append(checksum)

        print(f"   TX: {bytes(cmd).hex()}")
        self.ser.write(bytes(cmd))
        time.sleep(0.05)

        # Read any response
        if self.ser.in_waiting > 0:
            response = self.ser.read(self.ser.in_waiting)
            print(f"   RX: {response.hex()}")
            return response
        return None

    def parse_incoming_data(self, duration=3):
        """Parse incoming data packets"""
        print(f"\nListening for {duration} seconds and parsing packets...")

        buffer = bytearray()
        start_time = time.time()
        packets_found = 0

        while time.time() - start_time < duration:
            if self.ser.in_waiting > 0:
                buffer.extend(self.ser.read(self.ser.in_waiting))

        # Try to parse packets from buffer
        print(f"\nReceived {len(buffer)} bytes total: {buffer.hex()}")
        print("\nParsing packets...")

        i = 0
        while i < len(buffer):
            # Look for packet header
            if i + 2 < len(buffer) and buffer[i] == self.HEAD and buffer[i+1] == self.DEVICE_ID:
                if i + 2 < len(buffer):
                    length = buffer[i+2]
                    if i + length + 1 <= len(buffer):
                        packet = buffer[i:i+length+1]
                        print(f"\n  Packet #{packets_found + 1}: {packet.hex()}")
                        print(f"    Header: 0x{packet[0]:02x}")
                        print(f"    Device ID: 0x{packet[1]:02x}")
                        print(f"    Length: {packet[2]}")
                        if len(packet) > 3:
                            print(f"    Function: 0x{packet[3]:02x}")
                        packets_found += 1
                        i += length + 1
                        continue
            i += 1

        print(f"\nFound {packets_found} valid packets")
        return packets_found > 0

    def test_set_car_type(self):
        """Set car type to R2"""
        print("\n" + "="*60)
        print("Test: Setting car type to R2 (0x05)")
        print("="*60)

        cmd = [self.HEAD, self.DEVICE_ID, 0x04, self.FUNC_SET_CAR_TYPE, self.CARTYPE_R2]
        response = self.send_command(cmd)

        if response:
            print("✅ Car type set command sent and response received")
        else:
            print("⚠️  Command sent but no immediate response")

        time.sleep(0.5)
        return True

    def test_version_request(self):
        """Request firmware version"""
        print("\n" + "="*60)
        print("Test: Requesting firmware version")
        print("="*60)

        # Clear buffer
        if self.ser.in_waiting > 0:
            self.ser.read(self.ser.in_waiting)

        cmd = [self.HEAD, self.DEVICE_ID, 0x03, self.FUNC_VERSION]
        response = self.send_command(cmd)

        # Wait a bit more for response
        time.sleep(0.2)
        if self.ser.in_waiting > 0:
            response = self.ser.read(self.ser.in_waiting)
            print(f"   RX (delayed): {response.hex()}")

            # Try to parse version from response
            if len(response) >= 2:
                version = struct.unpack('H', response[:2])[0] / 100.0
                print(f"✅ Firmware version: {version}")
                return version

        print("❌ No version response received")
        return -1

    def test_beep(self):
        """Test beep command"""
        print("\n" + "="*60)
        print("Test: Beep command (listen for beep sound)")
        print("="*60)

        # Beep on
        cmd = [self.HEAD, self.DEVICE_ID, 0x04, self.FUNC_BEEP, 0x01]
        self.send_command(cmd)
        time.sleep(0.3)

        # Beep off
        cmd = [self.HEAD, self.DEVICE_ID, 0x04, self.FUNC_BEEP, 0x00]
        self.send_command(cmd)

        print("   Did you hear a beep? (If yes, board is responding!)")
        return True

    def test_motor_command(self):
        """Test motor control command"""
        print("\n" + "="*60)
        print("Test: Sending motor control command")
        print("Test: FORWARD at 0.2 m/s for 2 seconds")
        print("="*60)
        print("⚠️  WARNING: Robot wheels may move!")

        # Forward motion: vx=0.2, vy=0, vz=0
        vx = 0.2
        vy = 0.0
        vz = 0.0

        vx_bytes = bytearray(struct.pack('h', int(vx * 1000)))
        vy_bytes = bytearray(struct.pack('h', int(vy * 1000)))
        vz_bytes = bytearray(struct.pack('h', int(vz * 1000)))

        cmd = [self.HEAD, self.DEVICE_ID, 0x00, self.FUNC_MOTION, self.CARTYPE_R2,
               vx_bytes[0], vx_bytes[1], vy_bytes[0], vy_bytes[1], vz_bytes[0], vz_bytes[1]]
        cmd[2] = len(cmd) - 1  # Set length

        print(f"   Sending forward command (vx={vx}, vy={vy}, vz={vz})...")

        # Send command repeatedly for 2 seconds (like ROS does)
        start_time = time.time()
        count = 0
        while time.time() - start_time < 2.0:
            self.send_command(cmd)
            count += 1
            time.sleep(0.1)

        print(f"   Sent {count} motion commands")

        # Stop command
        print("   Sending STOP command...")
        vx_bytes = bytearray(struct.pack('h', 0))
        vy_bytes = bytearray(struct.pack('h', 0))
        vz_bytes = bytearray(struct.pack('h', 0))

        cmd = [self.HEAD, self.DEVICE_ID, 0x00, self.FUNC_MOTION, self.CARTYPE_R2,
               vx_bytes[0], vx_bytes[1], vy_bytes[0], vy_bytes[1], vz_bytes[0], vz_bytes[1]]
        cmd[2] = len(cmd) - 1

        self.send_command(cmd)

        print("\n   Did the motors move? Check the wheels!")
        return True

    def run_all_tests(self):
        """Run all diagnostic tests"""
        print("=" * 60)
        print("  ROS Master R2 Advanced Diagnostic")
        print("=" * 60)

        if not self.open_serial():
            return False

        try:
            # Test 1: Parse incoming data
            print("\n" + "="*60)
            print("Test 1: Parsing incoming auto-report data")
            print("="*60)
            self.parse_incoming_data(3)

            # Test 2: Set car type
            self.test_set_car_type()

            # Test 3: Request version
            version = self.test_version_request()

            # Test 4: Beep test
            self.test_beep()

            # Test 5: Motor command
            input("\nPress ENTER to test motor commands (or Ctrl+C to skip)...")
            self.test_motor_command()

            print("\n" + "="*60)
            print("  Diagnostic Complete!")
            print("="*60)

            if version > 0:
                print(f"\n✅ ROS Master board is communicating (firmware v{version})")
                print("   If motors didn't move, possible issues:")
                print("   - Motors not powered")
                print("   - Motor driver circuit issue")
                print("   - Battery voltage too low")
            else:
                print("\n⚠️  ROS Master board is sending data but version request failed")
                print("   Possible issues:")
                print("   - Packet synchronization problem")
                print("   - Firmware version issue")
                print("   - Need to restart the board")

        finally:
            self.close_serial()

        return True

if __name__ == "__main__":
    port = "/dev/ttyUSB1"

    if len(sys.argv) > 1:
        port = sys.argv[1]

    tester = RosmasterTester(port)
    tester.run_all_tests()
