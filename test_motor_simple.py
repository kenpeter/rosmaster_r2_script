#!/usr/bin/env python3
"""
Test YDLidar X4 motor - toggles motor on/off
"""
import serial
import time

port = '/dev/ydlidar'
baudrate = 128000

print(f"Opening {port} at {baudrate} baud...")
ser = serial.Serial(port, baudrate, timeout=1)

print("\n=== Testing Motor Control ===\n")

for cycle in range(3):
    print(f"Cycle {cycle + 1}/3:")

    # Motor OFF
    print("  Motor OFF (DTR=False)")
    ser.setDTR(False)
    time.sleep(3)

    # Motor ON
    print("  Motor ON (DTR=True) - SHOULD BE SPINNING NOW!")
    ser.setDTR(True)
    time.sleep(3)

    print()

# Final OFF
print("Final: Motor OFF")
ser.setDTR(False)
ser.close()

print("\n" + "="*60)
print("Test complete!")
print("="*60)
print("\nDid you feel/hear the motor spinning when DTR=True?")
print("The motor should spin for 3 seconds, then stop for 3 seconds,")
print("repeating 3 times total.")
