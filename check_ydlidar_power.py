#!/usr/bin/env python3
"""
Diagnose YDLidar power and connectivity
"""

import serial
import time
import subprocess
import os

print("\n" + "="*70)
print("  YDLidar X4 Power & Connectivity Diagnostic")
print("="*70 + "\n")

# Check 1: USB Device Detection
print("[1] USB Device Detection")
print("-" * 70)

try:
    result = subprocess.run(['lsusb'], capture_output=True, text=True)
    usb_devices = result.stdout

    # Look for common USB-Serial chips used by YDLidar
    if '1a86:7522' in usb_devices:
        print("✅ YDLidar USB device detected (CH340, ID 1a86:7522)")
    else:
        print("⚠️  YDLidar USB device NOT found")
        print("   Expected: USB ID 1a86:7522 (CH340 chip)")
        print("   Please check USB cable connection")

    print("\nAll USB devices:")
    for line in usb_devices.split('\n'):
        if '1a86' in line or 'Serial' in line:
            print(f"   {line}")

except Exception as e:
    print(f"❌ Error checking USB: {e}")

print()

# Check 2: Device Node
print("[2] Device Node Check")
print("-" * 70)

if os.path.exists('/dev/ydlidar'):
    result = subprocess.run(['ls', '-l', '/dev/ydlidar'], capture_output=True, text=True)
    print(f"✅ /dev/ydlidar exists")
    print(f"   {result.stdout.strip()}")

    # Check permissions
    if os.access('/dev/ydlidar', os.R_OK | os.W_OK):
        print("✅ Read/Write permissions OK")
    else:
        print("⚠️  Permission issue - you may need to add user to dialout group")
else:
    print("❌ /dev/ydlidar does NOT exist")
    print("   Check udev rules in /etc/udev/rules.d/")

print()

# Check 3: USB Power Information
print("[3] USB Power Status")
print("-" * 70)

try:
    # Find the USB device path
    result = subprocess.run(
        ['udevadm', 'info', '-a', '-n', '/dev/ydlidar'],
        capture_output=True, text=True
    )

    # Look for power info
    for line in result.stdout.split('\n'):
        if 'idVendor' in line or 'idProduct' in line or 'product' in line:
            print(f"   {line.strip()}")

    # Check USB bus power
    print("\n   USB Power Available:")
    usb_path = None
    for line in result.stdout.split('\n'):
        if 'KERNELS==' in line and 'usb' in line:
            kernels = line.split('"')[1]
            if '-' in kernels:  # USB device path like "1-2.4.2"
                usb_path = kernels.split(':')[0]
                break

    if usb_path:
        power_path = f"/sys/bus/usb/devices/{usb_path}/power"
        if os.path.exists(power_path):
            try:
                with open(f"{power_path}/runtime_status") as f:
                    print(f"   Runtime Status: {f.read().strip()}")
            except:
                pass

        # Try to read current draw (if available)
        curr_path = f"/sys/bus/usb/devices/{usb_path}/bMaxPower"
        if os.path.exists(curr_path):
            with open(curr_path) as f:
                max_power = f.read().strip()
                print(f"   Max Power: {max_power}")
    else:
        print("   ⚠️  Could not determine USB power info")

except Exception as e:
    print(f"   ⚠️  Could not read power info: {e}")

print()

# Check 4: Serial Communication Test
print("[4] Serial Communication Test")
print("-" * 70)

try:
    print("Opening serial port...")
    ser = serial.Serial('/dev/ydlidar', 128000, timeout=2)
    print("✅ Serial port opened successfully")

    # Test DTR control (motor power)
    print("\nTesting motor control (DTR pin)...")
    print("   Setting DTR = HIGH (motor ON)...")
    ser.setDTR(True)
    ser.setRTS(False)
    time.sleep(2)

    print("   ⏱️  Waiting 2 seconds for motor to spin up...")
    time.sleep(2)

    # Check for data
    print("\n   Reading data from lidar...")
    data_received = False
    for i in range(3):
        data = ser.read(200)
        if len(data) > 10:
            data_received = True
            print(f"   ✅ Received {len(data)} bytes")
            hex_str = ' '.join([f'{b:02x}' for b in data[:20]])
            print(f"      First 20 bytes: {hex_str}")
            break
        time.sleep(0.5)

    if not data_received:
        print("   ⚠️  No data received from lidar")
        print("      Possible causes:")
        print("      - Motor not spinning (insufficient power)")
        print("      - Motor needs external 5V power supply")
        print("      - Lidar malfunction")

    # Turn off motor
    ser.setDTR(False)
    ser.close()

except Exception as e:
    print(f"❌ Serial communication error: {e}")

print()

# Summary
print("="*70)
print("  DIAGNOSTIC SUMMARY")
print("="*70)
print()
print("Power Requirements for YDLidar X4:")
print("  • USB Data: 5V from USB port (for communication) ✓")
print("  • Motor Power: 5V, ~150-200mA (for spinning)")
print()
print("Power Sources:")
print("  1. USB Power Only (via DTR): If USB port provides enough current")
print("  2. External 5V: Connect separate 5V power to red wire")
print()
print("How to check if motor is spinning:")
print("  • Listen for whirring/buzzing sound from motor")
print("  • Look at the lidar - it should be rotating")
print("  • Data should be streaming when motor is ON")
print()
print("Expected behavior when working:")
print("  ✅ Motor spinning (audible)")
print("  ✅ Receiving >100 bytes of data per read")
print("  ✅ Data changes over time (not same bytes)")
print()

if data_received:
    print("🎉 YDLidar appears to be working!")
    print("   The motor is likely spinning and sending scan data.")
else:
    print("⚠️  YDLidar may need external power")
    print("   If motor is NOT spinning:")
    print("   1. Check if red wire is connected to 5V power")
    print("   2. Ensure power supply can provide 150-200mA")
    print("   3. Try connecting 5V from robot battery/power board")

print()
