#!/usr/bin/env python3
"""
Detailed signal analysis for YDLidar X4
Check DTR signal, RTS, and data flow
"""
import serial
import time

port = '/dev/ydlidar'
baudrate = 128000

print("="*60)
print("YDLidar X4 Detailed Signal Test")
print("="*60)
print()

# Open port
print("Opening serial port...")
ser = serial.Serial(port, baudrate, timeout=1)

# Check initial state
print("\n--- Initial Signal State ---")
print(f"DTR: {ser.dtr}")
print(f"RTS: {ser.rts}")
print(f"CTS: {ser.cts}")
print(f"DSR: {ser.dsr}")
print(f"RI: {ser.ri}")
print(f"CD: {ser.cd}")

# Test DTR control
print("\n--- Testing DTR Signal (Motor Control) ---")
print("Setting DTR = False (Motor should be OFF)")
ser.setDTR(False)
time.sleep(1)
print(f"DTR state: {ser.dtr}")
print("Waiting 2 seconds...")
time.sleep(2)

print("\nSetting DTR = True (Motor should turn ON)")
ser.setDTR(True)
time.sleep(1)
print(f"DTR state: {ser.dtr}")
print("*** MOTOR SHOULD BE SPINNING NOW ***")
print("Waiting 5 seconds - LISTEN/FEEL for motor!")
time.sleep(5)

print("\nSetting DTR = False (Motor should turn OFF)")
ser.setDTR(False)
time.sleep(1)
print(f"DTR state: {ser.dtr}")

# Check for any data coming in
print("\n--- Checking for incoming data ---")
print("Clearing buffer...")
ser.reset_input_buffer()

print("Enabling motor (DTR=True)...")
ser.setDTR(True)
time.sleep(1)

print("Sending SCAN command (A5 60)...")
ser.write(bytes([0xA5, 0x60]))
ser.flush()

print("Waiting 3 seconds for data...")
time.sleep(3)

bytes_available = ser.in_waiting
print(f"\nBytes in buffer: {bytes_available}")

if bytes_available > 0:
    data = ser.read(min(100, bytes_available))
    print(f"Received {len(data)} bytes:")
    print(f"Hex: {data.hex(' ')}")
    print(f"First 20 bytes: {' '.join(f'{b:02X}' for b in data[:20])}")
else:
    print("NO DATA RECEIVED!")

# Try GET_INFO command
print("\n--- Testing GET_INFO Command ---")
ser.reset_input_buffer()
print("Sending GET_INFO (A5 90)...")
ser.write(bytes([0xA5, 0x90]))
ser.flush()
time.sleep(0.5)

bytes_available = ser.in_waiting
print(f"Bytes in buffer: {bytes_available}")

if bytes_available > 0:
    data = ser.read(bytes_available)
    print(f"Response: {data.hex(' ')}")
else:
    print("NO RESPONSE!")

# Test reading raw data continuously
print("\n--- Continuous Data Monitor (5 seconds) ---")
ser.reset_input_buffer()
ser.setDTR(True)
ser.write(bytes([0xA5, 0x60]))  # SCAN command
ser.flush()

start = time.time()
total_bytes = 0
while time.time() - start < 5:
    if ser.in_waiting > 0:
        data = ser.read(ser.in_waiting)
        total_bytes += len(data)
        print(f"[{time.time()-start:.2f}s] {len(data):3d} bytes: {data.hex(' ')}")
    time.sleep(0.1)

print(f"\nTotal bytes received in 5 seconds: {total_bytes}")

# Cleanup
print("\n--- Cleanup ---")
ser.write(bytes([0xA5, 0x65]))  # STOP command
ser.setDTR(False)
ser.close()
print("Done!")

print("\n" + "="*60)
print("SUMMARY:")
print("  - If total bytes = 0: No communication at all")
print("  - If bytes > 0 but random: Sensor not working properly")
print("  - If motor didn't spin: Motor hardware broken")
print("="*60)
