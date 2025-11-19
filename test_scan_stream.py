#!/usr/bin/env python3
"""
Continuously read scan data stream from YDLidar X4
"""
import serial
import time

port = '/dev/ydlidar'
baudrate = 128000

print(f"Opening {port} at {baudrate} baud...")
ser = serial.Serial(port, baudrate, timeout=1)

# Enable motor
print("Enabling motor...")
ser.setDTR(True)
time.sleep(1)

# Clear buffer
ser.reset_input_buffer()

# Send scan command
print("Sending SCAN command...")
ser.write(bytes([0xA5, 0x60]))
ser.flush()

print("\nWaiting for scan data (10 seconds)...")
print("Reading continuous stream:\n")

start_time = time.time()
total_bytes = 0
packet_sync_found = False

while time.time() - start_time < 10:
    if ser.in_waiting > 0:
        # Read available data
        data = ser.read(min(ser.in_waiting, 100))
        total_bytes += len(data)

        # Print hex dump
        hex_str = ' '.join(f'{b:02X}' for b in data)
        print(f"[{time.time() - start_time:6.2f}s] {hex_str}")

        # Look for packet sync bytes
        if not packet_sync_found:
            if b'\xAA\x55' in data:
                print("   ^^^ FOUND PACKET SYNC (AA 55) ^^^")
                packet_sync_found = True
            elif b'\xA5\x5A' in data:
                print("   ^^^ FOUND RESPONSE HEADER (A5 5A) ^^^")
                packet_sync_found = True

    time.sleep(0.1)

print(f"\nTotal bytes received: {total_bytes}")
print(f"Packet sync found: {packet_sync_found}")

# Cleanup
ser.write(bytes([0xA5, 0x65]))  # STOP
ser.setDTR(False)
ser.close()
print("\nDone!")
