#!/usr/bin/env python3
"""
Test different YDLidar X4 command sequences
Based on X4 protocol documentation
"""
import serial
import time
import struct

port = '/dev/ydlidar'
baudrate = 128000

print(f"Opening {port} at {baudrate} baud...")
ser = serial.Serial(port, baudrate, timeout=1)

# Enable motor (DTR)
print("Enabling motor via DTR...")
ser.setDTR(True)
time.sleep(1)

# Try GET_INFO command first
print("\n1. Trying GET_INFO command (A5 90)...")
ser.write(bytes([0xA5, 0x90]))
ser.flush()
time.sleep(0.5)

if ser.in_waiting > 0:
    response = ser.read(ser.in_waiting)
    print(f"   Response ({len(response)} bytes): {response.hex(' ')}")
else:
    print("   No response")

# Try GET_HEALTH command
print("\n2. Trying GET_HEALTH command (A5 91)...")
ser.write(bytes([0xA5, 0x91]))
ser.flush()
time.sleep(0.5)

if ser.in_waiting > 0:
    response = ser.read(ser.in_waiting)
    print(f"   Response ({len(response)} bytes): {response.hex(' ')}")
else:
    print("   No response")

# Try STOP command to clear any previous state
print("\n3. Sending STOP command (A5 65)...")
ser.write(bytes([0xA5, 0x65]))
ser.flush()
time.sleep(0.5)

if ser.in_waiting > 0:
    response = ser.read(ser.in_waiting)
    print(f"   Response ({len(response)} bytes): {response.hex(' ')}")
else:
    print("   No response")

# Try SCAN command
print("\n4. Sending SCAN command (A5 60)...")
ser.write(bytes([0xA5, 0x60]))
ser.flush()
time.sleep(0.5)

print(f"   Bytes waiting: {ser.in_waiting}")

# Read first batch of scan data
if ser.in_waiting > 0:
    print("\n5. Reading scan data stream (first 200 bytes)...")
    data = ser.read(min(200, ser.in_waiting))
    print(f"   Data ({len(data)} bytes):")

    # Print as hex
    for i in range(0, len(data), 16):
        hex_str = ' '.join(f'{b:02X}' for b in data[i:i+16])
        print(f"   {i:04X}: {hex_str}")

    # Look for packet sync bytes
    print("\n   Looking for packet sync patterns:")
    print(f"   - AA 55 pattern (normal scan): ", end="")
    if b'\xAA\x55' in data:
        idx = data.index(b'\xAA\x55')
        print(f"FOUND at offset {idx}")
    else:
        print("NOT FOUND")

    print(f"   - A5 5A pattern (response header): ", end="")
    if b'\xA5\x5A' in data:
        idx = data.index(b'\xA5\x5A')
        print(f"FOUND at offset {idx}")
    else:
        print("NOT FOUND")
else:
    print("   No data received")

# Cleanup
print("\n6. Cleanup...")
ser.write(bytes([0xA5, 0x65]))  # STOP
ser.setDTR(False)  # Motor off
ser.close()
print("Done!")
