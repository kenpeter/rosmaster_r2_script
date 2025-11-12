#!/usr/bin/env python3
# Test script to diagnose ROS Master serial communication

import sys
import time
import serial

def test_serial_port(port="/dev/ttyUSB1"):
    """Test if we can open and communicate with the ROS Master board"""

    print("=" * 60)
    print("  ROS Master Serial Communication Test")
    print("=" * 60)
    print()

    # Test 1: Can we open the port?
    print(f"Test 1: Opening serial port {port}...")
    try:
        ser = serial.Serial(port, 115200, timeout=2)
        if ser.isOpen():
            print(f"✅ Serial port opened successfully!")
            print(f"   Port: {port}")
            print(f"   Baudrate: 115200")
        else:
            print(f"❌ Failed to open serial port")
            return False
    except Exception as e:
        print(f"❌ Error opening serial port: {e}")
        return False

    time.sleep(0.5)

    # Test 2: Check if there's any data coming in
    print()
    print("Test 2: Checking for incoming data (5 second timeout)...")
    print("        (The ROS Master should auto-report sensor data)")

    start_time = time.time()
    bytes_received = 0

    while time.time() - start_time < 5.0:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            bytes_received += len(data)
            print(f"   Received {len(data)} bytes: {data.hex()}")

    if bytes_received > 0:
        print(f"✅ Received {bytes_received} bytes total")
    else:
        print(f"❌ No data received! Possible issues:")
        print(f"   - ROS Master board not powered on")
        print(f"   - Wrong serial port selected")
        print(f"   - Faulty USB cable")
        print(f"   - ROS Master firmware not running")

    # Test 3: Try to send a version request command
    print()
    print("Test 3: Requesting firmware version...")

    # Clear any pending data
    if ser.in_waiting > 0:
        ser.read(ser.in_waiting)

    # Send version request command
    # Format: [HEAD, DEVICE_ID, LENGTH, FUNC_VERSION]
    HEAD = 0xFF
    DEVICE_ID = 0xFC
    FUNC_VERSION = 0x51
    COMPLEMENT = 257 - DEVICE_ID

    cmd = [HEAD, DEVICE_ID, 0x03, FUNC_VERSION]
    checksum = (sum(cmd) + COMPLEMENT) & 0xff
    cmd.append(checksum)

    print(f"   Sending command: {bytes(cmd).hex()}")
    ser.write(bytes(cmd))
    time.sleep(0.2)

    # Read response
    if ser.in_waiting > 0:
        response = ser.read(ser.in_waiting)
        print(f"✅ Received response: {response.hex()}")
    else:
        print(f"❌ No response to version request")

    # Test 4: Try to send a simple beep command
    print()
    print("Test 4: Sending beep command (you should hear a beep)...")

    FUNC_BEEP = 0x02
    cmd = [HEAD, DEVICE_ID, 0x04, FUNC_BEEP, 0x01]  # beep on
    checksum = (sum(cmd) + COMPLEMENT) & 0xff
    cmd.append(checksum)

    print(f"   Sending command: {bytes(cmd).hex()}")
    ser.write(bytes(cmd))
    time.sleep(0.5)

    # Turn beep off
    cmd = [HEAD, DEVICE_ID, 0x04, FUNC_BEEP, 0x00]  # beep off
    checksum = (sum(cmd) + COMPLEMENT) & 0xff
    cmd.append(checksum)
    ser.write(bytes(cmd))

    print("   If you heard a beep, the board is responding!")

    ser.close()

    print()
    print("=" * 60)
    print("  Test Complete")
    print("=" * 60)

    return True

if __name__ == "__main__":
    port = "/dev/ttyUSB1"

    if len(sys.argv) > 1:
        port = sys.argv[1]

    print(f"\nTesting serial port: {port}")
    print()

    test_serial_port(port)
