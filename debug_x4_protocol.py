#!/usr/bin/env python3
"""
Debug YDLidar X4 protocol - capture what it's actually sending
"""
import serial
import time
import sys

port = '/dev/ydlidar'
baudrate = 128000

print(f"Opening {port} at {baudrate} baud...")
ser = serial.Serial(port, baudrate, timeout=1)

# Enable motor
print("Enabling motor (DTR=True)...")
ser.setDTR(True)
time.sleep(1)

# Clear buffer
ser.reset_input_buffer()

# Send SCAN command (0xA5 0x60)
print("\nSending SCAN command (A5 60)...")
ser.write(bytes([0xA5, 0x60]))
ser.flush()

print("\nListening for response packets for 10 seconds...")
print("Looking for packet sync bytes: AA 55 (scan data) or A5 5A (response header)\n")

start_time = time.time()
buffer = bytearray()
packet_count = 0

while time.time() - start_time < 10:
    if ser.in_waiting > 0:
        data = ser.read(ser.in_waiting)
        buffer.extend(data)

        # Print raw hex
        hex_str = ' '.join(f'{b:02X}' for b in data)
        print(f"[{time.time() - start_time:6.2f}s] Received {len(data):3d} bytes: {hex_str}")

        # Look for AA 55 (scan packet header)
        while len(buffer) >= 2:
            if buffer[0] == 0xAA and buffer[1] == 0x55:
                print(f"\n*** FOUND SCAN PACKET HEADER (AA 55) at buffer position 0 ***")
                if len(buffer) >= 10:
                    print(f"    CT: {buffer[2]:02X}")
                    print(f"    LSN: {buffer[3]:02X} ({buffer[3]} samples)")
                    fsa = (buffer[5] << 8) | buffer[4]
                    lsa = (buffer[7] << 8) | buffer[6]
                    print(f"    FSA: {fsa:04X} ({(fsa >> 1) / 64.0:.2f} degrees)")
                    print(f"    LSA: {lsa:04X} ({(lsa >> 1) / 64.0:.2f} degrees)")
                    print(f"    CS:  {buffer[9]:02X}{buffer[8]:02X}")
                    packet_count += 1
                    # Remove this packet from buffer
                    buffer = buffer[10:]
                else:
                    break
            else:
                buffer.pop(0)

    time.sleep(0.05)

print(f"\n" + "="*60)
print(f"Summary:")
print(f"  Total scan packets found: {packet_count}")
print(f"  Remaining buffer: {len(buffer)} bytes")
if buffer:
    print(f"  Buffer hex: {buffer.hex(' ')}")

# Cleanup
print("\nSending STOP command (A5 65)...")
ser.write(bytes([0xA5, 0x65]))
ser.setDTR(False)
ser.close()
print("Done!")
