#!/usr/bin/env python3
"""
Raw YDLidar X4 data reader to see what the device is actually sending
"""
import serial
import time

port = '/dev/ydlidar'
baudrate = 128000

print(f"Opening {port} at {baudrate} baud...")
ser = serial.Serial(port, baudrate, timeout=1)

# Enable motor
ser.setDTR(True)
time.sleep(0.5)

# Send scan command
print("Sending scan command...")
ser.write(bytes([0xA5, 0x60]))  # SCAN command
ser.flush()

print("\nReading raw data (first 500 bytes):")
print("=" * 60)

for i in range(500):
    if ser.in_waiting > 0:
        byte = ser.read(1)
        print(f"{byte[0]:02X}", end=" ")
        if (i + 1) % 16 == 0:
            print()
    else:
        time.sleep(0.01)

print("\n" + "=" * 60)
print("Done!")

# Stop and cleanup
ser.write(bytes([0xA5, 0x65]))  # STOP command
ser.setDTR(False)
ser.close()
