#!/usr/bin/env python3
"""
RPLiDAR Motor Control Tool
Tests and controls the RPLiDAR motor using DTR signal
"""

import serial
import time
import sys

def test_motor_control(port='/dev/rplidar', baudrate=115200):
    """Test RPLiDAR motor control via DTR signal"""

    print("=" * 60)
    print("  RPLiDAR Motor Control Test")
    print("=" * 60)
    print(f"\nPort: {port}")
    print(f"Baudrate: {baudrate}")
    print()

    try:
        # Open serial port
        print("Opening serial port...")
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=1,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        print("✅ Serial port opened successfully")
        print()

        # Test 1: Motor ON (DTR = False)
        print("Test 1: Turning motor ON (DTR=False)")
        print("        You should hear the motor start spinning...")
        ser.setDTR(False)  # DTR low = motor ON for RPLiDAR
        time.sleep(2)

        if input("\nDid the motor start spinning? (y/n): ").lower() == 'y':
            print("✅ Motor control working!")
        else:
            print("⚠️  Motor did not spin")
            print("\nPossible issues:")
            print("1. Hardware problem with RPLiDAR motor")
            print("2. Insufficient power (check USB power)")
            print("3. Faulty USB cable")
            print("4. RPLiDAR may be damaged")

        print()

        # Test 2: Motor OFF (DTR = True)
        print("Test 2: Turning motor OFF (DTR=True)")
        ser.setDTR(True)  # DTR high = motor OFF
        time.sleep(1)
        print("Motor should have stopped")
        print()

        # Test 3: Motor ON again
        print("Test 3: Turning motor ON again")
        ser.setDTR(False)
        time.sleep(2)

        print()
        print("=" * 60)
        print("  Keeping motor ON for testing...")
        print("  Press Ctrl+C to stop")
        print("=" * 60)
        print()

        # Keep motor running
        while True:
            time.sleep(1)

    except serial.SerialException as e:
        print(f"\n❌ Serial port error: {e}")
        print("\nTroubleshooting:")
        print("1. Check if /dev/rplidar exists")
        print("2. Check permissions: ls -la /dev/rplidar")
        print("3. Try: sudo chmod 666 /dev/rplidar")
        return False

    except KeyboardInterrupt:
        print("\n\nStopping motor...")
        ser.setDTR(True)  # Turn off motor
        ser.close()
        print("✅ Motor stopped, port closed")
        return True

    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        return False

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/rplidar'
    test_motor_control(port)
