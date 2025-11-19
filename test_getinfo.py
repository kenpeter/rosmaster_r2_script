#!/usr/bin/env python3
"""
Test YDLidar X4 GET_INFO command
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

# Clear buffer
ser.reset_input_buffer()

# Send GET_INFO command (A5 90)
print("Sending GET_INFO command (A5 90)...")
ser.write(bytes([0xA5, 0x90]))
ser.flush()
time.sleep(0.3)

if ser.in_waiting > 0:
    response = ser.read(ser.in_waiting)
    print(f"\nReceived {len(response)} bytes:")
    print(f"Hex: {response.hex(' ')}")

    # Check for response header A5 5A
    if len(response) >= 7 and response[0] == 0xA5 and response[1] == 0x5A:
        print("\n✓ Valid response header (A5 5A) found!")
        length = (response[3] << 8) | response[2]
        print(f"  Response length: {length} bytes")
        print(f"  Type: 0x{response[6]:02X}")

        if len(response) >= 20:
            print(f"\nDevice Info:")
            print(f"  Model: {response[7]}")
            print(f"  Firmware: {response[8]}.{response[9]}")
            print(f"  Hardware: {response[10]}")
            print(f"  Serial: {response[11:19].hex()}")
    else:
        print("\n✗ No valid response header found")
        print("  Response doesn't match expected format")
else:
    print("\n✗ No response received")

# Cleanup
ser.setDTR(False)
ser.close()
print("\nDone!")
