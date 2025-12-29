#!/usr/bin/env python3
"""
YDLIDAR TG30 Test Script
Tests the YDLIDAR connection and reads basic data
"""

import serial
import time
import sys
import struct

def main():
    print("="*60)
    print("  YDLIDAR TG30 CONNECTION TEST")
    print("="*60)

    lidar_port = "/dev/ydlidar"
    baudrate = 512000  # TG30 correct baudrate

    print(f"\nConnecting to YDLIDAR...")
    print(f"  Port: {lidar_port}")
    print(f"  Baudrate: {baudrate}")

    try:
        # Open serial connection
        ser = serial.Serial(
            port=lidar_port,
            baudrate=baudrate,
            timeout=2,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        print("✅ Serial port opened successfully!")

        # Clear buffer
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.1)

        # Test 1: Check if port is readable
        print("\n[TEST 1] Checking serial connection...")
        if ser.is_open:
            print("✅ Port is open and ready")
        else:
            print("❌ Port failed to open")
            sys.exit(1)

        # Test 2: Send start scan command
        print("\n[TEST 2] Sending START SCAN command...")
        # YDLIDAR protocol: Start scan command
        # Command format: A5 60 (start motor and scan)
        start_cmd = bytes([0xA5, 0x60])
        ser.write(start_cmd)
        print("✅ Start command sent")
        time.sleep(0.5)

        # Test 3: Try to read data
        print("\n[TEST 3] Reading LIDAR data (10 seconds)...")
        print("👀 Watch for scan data packets...")
        print("-"*60)

        start_time = time.time()
        packet_count = 0
        byte_count = 0
        error_count = 0

        while time.time() - start_time < 10:
            if ser.in_waiting > 0:
                try:
                    # Read available data
                    data = ser.read(ser.in_waiting)
                    byte_count += len(data)

                    # Look for packet headers (0xAA 0x55 is common YDLIDAR header)
                    for i in range(len(data) - 1):
                        if data[i] == 0xAA and data[i+1] == 0x55:
                            packet_count += 1

                    # Display progress every second
                    elapsed = int(time.time() - start_time)
                    print(f"  [{elapsed}s] Bytes received: {byte_count}, Packets detected: {packet_count}", end='\r')

                except Exception as e:
                    error_count += 1
                    if error_count < 5:
                        print(f"\n⚠️  Read error: {e}")

            time.sleep(0.01)

        print("\n" + "-"*60)

        # Test 4: Send stop command
        print("\n[TEST 4] Sending STOP SCAN command...")
        stop_cmd = bytes([0xA5, 0x65])
        ser.write(stop_cmd)
        print("✅ Stop command sent")

        # Close port
        ser.close()
        print("\n✅ Serial port closed")

        # Results
        print("\n" + "="*60)
        print("  TEST RESULTS")
        print("="*60)
        print(f"\n📊 Statistics:")
        print(f"   Total bytes received: {byte_count}")
        print(f"   Packets detected: {packet_count}")
        print(f"   Read errors: {error_count}")

        if byte_count > 1000 and packet_count > 10:
            print("\n✅ SUCCESS! LIDAR is communicating properly!")
            print("   - Receiving data packets")
            print("   - Motor appears to be spinning")
            print("   - Device is working correctly")
        elif byte_count > 0:
            print("\n⚠️  PARTIAL SUCCESS:")
            print("   - Some data received but less than expected")
            print("   - Check if LIDAR motor is spinning")
            print("   - May need to adjust baudrate or check connections")
        else:
            print("\n❌ FAILED: No data received from LIDAR")
            print("\nTroubleshooting:")
            print("   1. Check LIDAR USB connection")
            print("   2. Check LIDAR power (red LED should be on)")
            print("   3. Listen for motor spinning sound")
            print("   4. Check device: ls -la /dev/ydlidar")
            print("   5. Try: sudo chmod 666 /dev/ydlidar")

        print("\n" + "="*60)
        print("  YDLIDAR TEST COMPLETE")
        print("="*60)

    except serial.SerialException as e:
        print(f"\n❌ Serial port error: {e}")
        print("\nPossible issues:")
        print("   - LIDAR is not connected")
        print("   - Wrong device (/dev/ydlidar)")
        print("   - Device in use by another process")
        print("   - Permission denied")
        print("\nTry:")
        print("   sudo chmod 666 /dev/ydlidar")
        print("   ls -la /dev/ydlidar /dev/ttyUSB*")
        sys.exit(1)

    except KeyboardInterrupt:
        print("\n\n🛑 Test stopped by user (Ctrl+C)")
        if 'ser' in locals() and ser.is_open:
            ser.close()
        sys.exit(0)

    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        if 'ser' in locals() and ser.is_open:
            ser.close()
        sys.exit(1)

if __name__ == "__main__":
    main()
