#!/usr/bin/env python3
"""
Test YDLidar X4 basic functionality
"""

import serial
import time

print("\n" + "="*60)
print("  YDLidar X4 Test")
print("="*60 + "\n")

port = '/dev/ydlidar'
baudrate = 128000  # YDLidar X4 typically uses 128000

try:
    print(f"Opening {port} at {baudrate} baud...")
    ser = serial.Serial(port, baudrate, timeout=2)
    print("✅ Connected successfully\n")

    time.sleep(1)

    # Try to read some data
    print("Reading data from lidar...")
    print("(The lidar motor should be spinning)\n")

    for i in range(5):
        data = ser.read(500)
        if len(data) > 0:
            print(f"[{i+1}] Received {len(data)} bytes")
            # Show first few bytes
            hex_str = ' '.join([f'{b:02x}' for b in data[:30]])
            print(f"    First 30 bytes: {hex_str}")
        else:
            print(f"[{i+1}] No data received")
        time.sleep(0.5)

    ser.close()

    print("\n" + "="*60)
    print("  Test Complete")
    print("="*60)
    print("\n✅ If you saw data above, the YDLidar hardware is working!")
    print("⚠️  If no data, check:")
    print("   - Is the lidar motor spinning?")
    print("   - Is the USB cable properly connected?")
    print("   - Does the lidar have power?\n")

except Exception as e:
    print(f"\n❌ Error: {e}")
    import traceback
    traceback.print_exc()
