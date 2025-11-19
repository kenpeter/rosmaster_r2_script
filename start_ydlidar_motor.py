#!/usr/bin/env python3
"""
Start and keep the YDLidar motor spinning via DTR control.
Run this in the background before launching the ROS driver.
"""

import serial
import time
import signal
import sys

running = True

def signal_handler(sig, frame):
    global running
    print('\n\nStopping motor...')
    running = False

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

print("="*60)
print("  YDLidar Motor Control")
print("="*60)
print("\nStarting motor via DTR pin control...")

try:
    ser = serial.Serial('/dev/ydlidar', 128000, timeout=1)

    # Enable motor via DTR
    ser.setDTR(True)
    ser.setRTS(False)

    print("✅ Motor power enabled (DTR=HIGH)")
    print("\nListening... The motor should be spinning now.")
    print("Check if you can hear the motor spinning.")
    print("\nPress Ctrl+C to stop the motor.\n")

    # Keep the port open and DTR high
    while running:
        time.sleep(1)

    # Turn off motor
    ser.setDTR(False)
    ser.close()
    print("Motor stopped.")

except KeyboardInterrupt:
    print("\nStopping...")
    if 'ser' in locals():
        ser.setDTR(False)
        ser.close()
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()
