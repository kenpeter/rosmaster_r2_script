#!/usr/bin/env python3
"""
YDLIDAR X4 Diagnostic Tool
Tests different ports and settings to find the correct configuration
"""

import serial
import time
import sys

def test_port(port_name, baudrate):
    """Test a specific port with given baudrate"""
    print(f"\n{'='*60}")
    print(f"Testing: {port_name} at {baudrate} baud")
    print(f"{'='*60}")

    try:
        port = serial.Serial(port_name, baudrate, timeout=2)
        print(f"✅ Port opened successfully")

        # Test DTR control (motor control)
        print("\n1. Testing motor control (DTR signal)...")
        port.setDTR(False)  # Motor ON
        print("   Motor ON (DTR=Low)")
        time.sleep(2)

        # Send device info command (YDLIDAR X4 protocol)
        print("\n2. Sending device info request...")
        # Command: 0xA5 0x90 (Get Device Info)
        port.write(bytes([0xA5, 0x90]))
        time.sleep(0.1)

        # Read response
        response = port.read(100)
        if response:
            print(f"   ✅ Received {len(response)} bytes")
            print(f"   Data (hex): {response[:min(50, len(response))].hex()}")

            # Check for YDLIDAR response signature
            if 0xA5 in response:
                print(f"   ✅ Found YDLIDAR signature (0xA5)")
                port.setDTR(True)  # Motor OFF
                port.close()
                return True
        else:
            print(f"   ⚠️  No response to device info command")

        # Try reading raw data
        print("\n3. Reading raw scan data...")
        port.setDTR(False)  # Make sure motor is on
        time.sleep(1)

        data = port.read(200)
        if data:
            print(f"   ✅ Received {len(data)} bytes of scan data")
            print(f"   First 50 bytes (hex): {data[:min(50, len(data))].hex()}")

            # Check for scan packet header (0xAA 0x55)
            if b'\xAA\x55' in data or b'\xA5\x5A' in data:
                print(f"   ✅ Found scan packet header!")
                port.setDTR(True)  # Motor OFF
                port.close()
                return True
        else:
            print(f"   ⚠️  No scan data received")

        port.setDTR(True)  # Motor OFF
        port.close()

    except serial.SerialException as e:
        print(f"   ❌ Serial error: {e}")
        return False
    except Exception as e:
        print(f"   ❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return False

    return False

def main():
    print("="*60)
    print("  YDLIDAR X4 Diagnostic Tool")
    print("="*60)

    # Test different configurations
    configs = [
        ('/dev/ttyUSB0', 128000),
        ('/dev/ttyUSB1', 128000),
        ('/dev/ydlidar', 128000),
        ('/dev/ttyUSB0', 115200),
        ('/dev/ttyUSB1', 115200),
        ('/dev/ttyUSB0', 230400),
        ('/dev/ttyUSB1', 230400),
    ]

    working_configs = []

    for port, baudrate in configs:
        result = test_port(port, baudrate)
        if result:
            working_configs.append((port, baudrate))
            print(f"\n{'='*60}")
            print(f"✅ SUCCESS! Working configuration found:")
            print(f"   Port: {port}")
            print(f"   Baudrate: {baudrate}")
            print(f"{'='*60}")

    print("\n" + "="*60)
    print("  DIAGNOSTIC SUMMARY")
    print("="*60)

    if working_configs:
        print("\n✅ Found working configuration(s):")
        for port, baudrate in working_configs:
            print(f"   - {port} @ {baudrate} baud")
        print("\nUpdate your ydlidar_x4.yaml with these settings!")
    else:
        print("\n❌ No working configuration found")
        print("\nPossible issues:")
        print("  1. Wrong USB port - YDLIDAR may be on different port")
        print("  2. Hardware issue - YDLIDAR may be defective")
        print("  3. Power issue - YDLIDAR motor not getting enough power")
        print("  4. Wrong baud rate - try other common rates")

    print("="*60)

if __name__ == "__main__":
    main()
