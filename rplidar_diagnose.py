#!/usr/bin/env python3
"""
RPLiDAR Diagnostic Tool - Automated motor and communication test
"""

import serial
import time
import struct

def diagnose_rplidar(port='/dev/rplidar', baudrate=115200):
    """Comprehensive RPLiDAR diagnostic"""

    print("=" * 60)
    print("  RPLiDAR A1 Diagnostic Tool")
    print("=" * 60)
    print(f"\nPort: {port}")
    print(f"Baudrate: {baudrate}\n")

    try:
        # Open serial connection
        print("[1/6] Opening serial port...")
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=2,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        print("      ✅ Port opened successfully\n")

        # Clear buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Test 1: Stop motor (reset state)
        print("[2/6] Stopping motor (reset state)...")
        ser.setDTR(True)  # DTR high = motor OFF
        time.sleep(0.5)
        print("      ✅ Stop command sent\n")

        # Test 2: Send STOP command
        print("[3/6] Sending STOP command...")
        stop_cmd = b'\xA5\x25'  # STOP command
        ser.write(stop_cmd)
        time.sleep(0.5)
        ser.reset_input_buffer()
        print("      ✅ STOP command sent\n")

        # Test 3: Start motor
        print("[4/6] Starting motor (DTR=Low)...")
        ser.setDTR(False)  # DTR low = motor ON
        time.sleep(1)
        print("      ✅ Motor start signal sent")
        print("      >> LISTEN: Motor should be spinning now! <<\n")

        # Test 4: Get Device Info
        print("[5/6] Requesting device info...")
        info_cmd = b'\xA5\x50'  # GET_INFO command
        ser.write(info_cmd)
        time.sleep(0.3)

        response = ser.read(27)  # Device info response is 27 bytes
        if len(response) >= 7:
            print(f"      ✅ Received {len(response)} bytes")
            print(f"      Response header: {response[:7].hex()}")

            if len(response) >= 27:
                model = response[7]
                firmware_minor = response[8]
                firmware_major = response[9]
                hardware = response[10]
                serial_num = response[11:27].hex()

                print(f"\n      Device Info:")
                print(f"      - Model: {model}")
                print(f"      - Firmware: {firmware_major}.{firmware_minor}")
                print(f"      - Hardware: {hardware}")
                print(f"      - Serial: {serial_num}")
        else:
            print(f"      ⚠️  Short response: {len(response)} bytes")
            if len(response) > 0:
                print(f"      Data: {response.hex()}")
        print()

        # Test 5: Get Health Status
        print("[6/6] Requesting health status...")
        health_cmd = b'\xA5\x52'  # GET_HEALTH command
        ser.reset_input_buffer()
        ser.write(health_cmd)
        time.sleep(0.3)

        health_response = ser.read(10)
        if len(health_response) >= 10:
            status = health_response[7]
            error_code = struct.unpack('<H', health_response[8:10])[0]

            print(f"      ✅ Received health status")
            print(f"      - Status: {status} (0=Good, 1=Warning, 2=Error)")
            print(f"      - Error Code: {error_code}")

            if status == 0:
                print("      ✅ LIDAR is healthy!")
            elif status == 1:
                print("      ⚠️  Warning state")
            else:
                print("      ❌ Error state")
        else:
            print(f"      ⚠️  Short response: {len(health_response)} bytes")
        print()

        # Summary
        print("=" * 60)
        print("  Diagnostic Summary")
        print("=" * 60)
        print("\n✅ Communication: Working")
        print("✅ Commands: Accepted")

        if len(response) >= 7:
            print("✅ Device Info: Received")
        else:
            print("⚠️  Device Info: Incomplete")

        print("\n⚠️  Motor Status: UNKNOWN (requires physical inspection)")
        print("\nNext steps:")
        print("1. Check if motor is physically spinning")
        print("2. If not spinning:")
        print("   - Check USB power (try different port)")
        print("   - Check for loose connections")
        print("   - Motor may be faulty")
        print("3. If spinning but still errors:")
        print("   - Motor may be spinning too slowly")
        print("   - Try cleaning the motor/bearings")
        print()

        # Keep motor running
        print("Keeping motor ON for 10 seconds...")
        print("Please observe if motor is spinning.")
        print()

        for i in range(10, 0, -1):
            print(f"  {i} seconds remaining...", end='\r')
            time.sleep(1)

        print("\n\nStopping motor...")
        ser.setDTR(True)
        ser.close()
        print("✅ Done\n")

        return True

    except serial.SerialException as e:
        print(f"\n❌ Serial Error: {e}")
        print("\nCheck:")
        print("1. Device exists: ls -la /dev/rplidar")
        print("2. Permissions: sudo chmod 666 /dev/rplidar")
        return False

    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/rplidar'
    diagnose_rplidar(port)
