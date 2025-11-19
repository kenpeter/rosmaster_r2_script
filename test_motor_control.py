#!/usr/bin/env python3
"""
Test YDLidar X4 motor control via DTR
"""
import serial
import time

port = '/dev/ydlidar'
baudrate = 128000

print(f"Opening {port} at {baudrate} baud...")
ser = serial.Serial(port, baudrate, timeout=1)

print("\n=== Motor Control Test ===\n")

# Motor OFF
print("1. Motor OFF (DTR=False)...")
ser.setDTR(False)
time.sleep(2)
input("   Touch the lidar. Is it spinning? Press Enter...")

# Motor ON
print("\n2. Motor ON (DTR=True)...")
ser.setDTR(True)
time.sleep(2)
input("   Touch the lidar now. Is it spinning? Press Enter...")

# Motor OFF again
print("\n3. Motor OFF again (DTR=False)...")
ser.setDTR(False)
time.sleep(1)
print("   Motor should stop now")

ser.close()
print("\nTest complete!")
print("\nIf the motor NEVER spun, there may be:")
print("  - A power issue (motor needs external 5V)")
print("  - A faulty motor")
print("  - Incorrect wiring")
