#!/usr/bin/env python3
"""
Minimal Motor Test Script
Tests the robot's drive motors (forward, backward, stop)
"""

import sys
import time
sys.path.append('/opt/ros/humble/lib/python3.10/site-packages')

from Rosmaster_Lib import Rosmaster

def main():
    print("="*60)
    print("  MOTOR TEST - Rosmaster R2")
    print("="*60)

    # Initialize the robot
    print("\nInitializing Rosmaster R2...")
    car = Rosmaster()
    car.set_car_type(5)  # R2 = 5

    # Start the serial receive thread (required for data updates)
    car.create_receive_threading()

    # Enable auto-report to get real-time data (including battery voltage)
    car.set_auto_report_state(True, forever=False)
    time.sleep(1.0)  # Wait for data to arrive

    # Check battery voltage
    voltage = car.get_battery_voltage()
    print(f"🔋 Battery Voltage: {voltage} V")
    if voltage < 9.0:
        print("⚠️  WARNING: Voltage is too low! Motors may not move.")
        print("   Check power switch or charge battery.")


    try:
        # Test 1: Stop (safety)
        print("\n[TEST 1] Stopping motors (safety)...")
        car.set_car_motion(0, 0, 0)
        time.sleep(1)
        print("✅ Motors stopped")

        # Test 2: Forward
        print("\n[TEST 2] Moving FORWARD (30% speed for 3 seconds)...")
        print("👀 Watch the robot - it should move forward!")
        car.set_car_motion(0.3, 0, 0)  # vx=0.3 m/s forward
        time.sleep(3)
        car.set_car_motion(0, 0, 0)  # Stop
        print("✅ Forward motion complete")
        time.sleep(1)

        # Test 3: Backward
        print("\n[TEST 3] Moving BACKWARD (30% speed for 3 seconds)...")
        print("👀 Watch the robot - it should move backward!")
        car.set_car_motion(-0.3, 0, 0)  # vx=-0.3 m/s backward
        time.sleep(3)
        car.set_car_motion(0, 0, 0)  # Stop
        print("✅ Backward motion complete")
        time.sleep(1)

        # Test 4: Rotation
        print("\n[TEST 4] Rotating LEFT (3 seconds)...")
        print("👀 Watch the robot - it should rotate!")
        car.set_car_motion(0, 0, 0.5)  # rotation
        time.sleep(3)
        car.set_car_motion(0, 0, 0)  # Stop
        print("✅ Rotation complete")

        # Final stop
        print("\n[FINAL] Stopping all motors...")
        car.set_car_motion(0, 0, 0)

        print("\n" + "="*60)
        print("  MOTOR TEST COMPLETE!")
        print("="*60)
        print("\n✅ If motors moved, hardware is working!")
        print("❌ If motors didn't move:")
        print("   - Check battery connection")
        print("   - Check motor driver power")
        print("   - Check serial connection (/dev/ttyUSB*)")

    except Exception as e:
        print(f"\n❌ Error: {e}")
        car.set_car_motion(0, 0, 0)  # Safety stop
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        # Disable auto-report and cleanup
        car.set_auto_report_state(False, forever=False)

if __name__ == "__main__":
    main()
