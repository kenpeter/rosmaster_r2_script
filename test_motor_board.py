#!/usr/bin/env python3
"""
Direct Motor Board Test - Bypasses ROS to test hardware directly
"""

import serial
import time
import sys

def find_motor_board():
    """Find the motor controller serial port"""
    for port in ['/dev/ttyUSB2', '/dev/ttyUSB1', '/dev/ttyUSB0']:
        try:
            ser = serial.Serial(port, 115200, timeout=1)
            time.sleep(0.5)
            # Read some data
            data = ser.read(100)
            ser.close()

            # ROSMaster board sends data with 0xff 0xfb signature
            if b'\xff\xfb' in data and len(data) > 30:
                return port
        except:
            continue
    return None

print("\n" + "="*60)
print("  DIRECT MOTOR BOARD TEST")
print("="*60 + "\n")

# Find motor board
print("🔍 Searching for motor controller board...")
port = find_motor_board()

if not port:
    print("❌ Motor controller board not found!")
    print("   Checked: /dev/ttyUSB0, /dev/ttyUSB1, /dev/ttyUSB2")
    sys.exit(1)

print(f"✅ Found motor board on {port}\n")

try:
    ser = serial.Serial(port, 115200, timeout=2)
    print(f"📡 Opened serial connection: {port} @ 115200 baud\n")

    # Read initial data
    print("📊 Reading sensor data from board...")
    time.sleep(0.5)
    ser.reset_input_buffer()
    time.sleep(0.2)

    data = ser.read(500)
    print(f"   Received {len(data)} bytes")

    if len(data) < 20:
        print("⚠️  WARNING: Very little data received!")
        print("   The board might not be responding properly.\n")
    else:
        print("✅ Board is sending data\n")

    # Try to parse voltage from data
    print("🔋 Attempting to read battery voltage from raw data...")
    if len(data) > 50:
        # ROSMaster sends voltage in the data stream
        # Look for voltage bytes (usually around index 20-30)
        for i in range(len(data)-4):
            if data[i] == 0xff and data[i+1] == 0xfb:
                # Found packet start
                if i+20 < len(data):
                    voltage_raw = (data[i+18] << 8) | data[i+19]
                    voltage = voltage_raw / 100.0
                    if 0 < voltage < 20:
                        print(f"   Raw voltage reading: {voltage:.2f}V")
                        if voltage < 0.5:
                            print("   ⚠️  Voltage sensor might be disconnected!")
                        break

    print("\n" + "="*60)
    print("  MOTOR COMMAND TEST")
    print("="*60 + "\n")

    print("🎮 Sending motor commands...\n")

    # Motor command format for ROSMaster:
    # [0xff, 0xfe, length, cmd, data...]

    # Command 1: Stop all motors (safe command)
    print("1️⃣  Stopping all motors (safety)...")
    cmd_stop = bytes([0xff, 0xfe, 0x0c, 0x00, 0x01,
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xed])
    ser.write(cmd_stop)
    time.sleep(0.5)
    print("   ✅ Stop command sent\n")

    # Command 2: Small forward movement
    print("2️⃣  Testing FORWARD movement (50% speed for 2 seconds)...")
    print("   👀 WATCH YOUR MOTORS - they should move now!\n")

    # Forward: speed = 127 (50% of 255)
    cmd_forward = bytes([0xff, 0xfe, 0x0c, 0x00, 0x01,
                         0x7f, 0x00, 0x7f, 0x00, 0x00, 0x00, 0x6c])

    ser.write(cmd_forward)
    print("   📤 Forward command sent - motors should spin NOW!")
    time.sleep(2)

    # Stop
    ser.write(cmd_stop)
    print("   🛑 Stop command sent\n")
    time.sleep(1)

    # Command 3: Backward movement
    print("3️⃣  Testing BACKWARD movement (50% speed for 2 seconds)...")
    cmd_backward = bytes([0xff, 0xfe, 0x0c, 0x00, 0x01,
                          0x00, 0x7f, 0x00, 0x7f, 0x00, 0x00, 0xed])

    ser.write(cmd_backward)
    print("   📤 Backward command sent - motors should spin NOW!")
    time.sleep(2)

    # Stop
    ser.write(cmd_stop)
    print("   🛑 Stop command sent\n")

    ser.close()

    print("="*60)
    print("  TEST COMPLETE")
    print("="*60 + "\n")

    print("❓ DID THE MOTORS MOVE?\n")
    print("  If YES:")
    print("    • Hardware is working!")
    print("    • Problem is in ROS communication chain")
    print("    • Need to restart the driver node\n")

    print("  If NO:")
    print("    • Motor board blue LEDs are on = power OK")
    print("    • Board receives commands = serial OK")
    print("    • But motors don't move = BRAKE/LOCK engaged!")
    print("    • Check for:")
    print("      - Emergency stop button pressed")
    print("      - Motor enable switch/jumper")
    print("      - Motor brake engaged")
    print("      - Motor driver in fault mode\n")

except serial.SerialException as e:
    print(f"❌ Serial Error: {e}")
    print("   The port might be in use by another process.")
    print("   Try stopping the ROS launch file first.")
    sys.exit(1)

except Exception as e:
    print(f"❌ Error: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
