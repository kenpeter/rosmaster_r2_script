#!/usr/bin/env python3
"""
Minimal Steering Test Script (Ackermann Front Wheel)
Tests the front wheel servo steering for R2 robot
"""

import sys
import time
sys.path.append('/opt/ros/humble/lib/python3.10/site-packages')

from Rosmaster_Lib import Rosmaster

def main():
    print("="*60)
    print("  STEERING TEST - Ackermann Front Wheel (R2)")
    print("="*60)

    # Initialize the robot
    print("\nInitializing Rosmaster R2...")
    car = Rosmaster()
    car.set_car_type(5)  # R2 = 5 (Ackermann type)

    # Start the serial receive thread (required for data updates)
    car.create_receive_threading()

    # Enable auto-report to get real-time data
    car.set_auto_report_state(True, forever=False)
    time.sleep(1.0)  # Wait for data to arrive

    # Check battery voltage
    voltage = car.get_battery_voltage()
    print(f"🔋 Battery Voltage: {voltage} V")

    try:
        # Test 1: Get current default angle
        print("\n[TEST 1] Reading current default steering angle...")
        try:
            default_angle = car.get_akm_default_angle()
            print(f"✅ Default steering angle: {default_angle}°")
        except Exception as e:
            print(f"⚠️  Could not read default angle: {e}")
            default_angle = 90  # Assume center

        # Test 2: Center position
        print("\n[TEST 2] Setting steering to CENTER (0°)...")
        print("👀 Watch the front wheels - they should center")
        car.set_akm_steering_angle(0)
        time.sleep(2)
        print("✅ Center position set")

        # Test 3: Turn left
        print("\n[TEST 3] Turning steering LEFT (-30°)...")
        print("👀 Watch the front wheels - they should turn LEFT")
        car.set_akm_steering_angle(-30)
        time.sleep(2)
        print("✅ Left turn complete")

        # Test 4: Back to center
        print("\n[TEST 4] Returning to CENTER (0°)...")
        car.set_akm_steering_angle(0)
        time.sleep(2)
        print("✅ Back to center")

        # Test 5: Turn right
        print("\n[TEST 5] Turning steering RIGHT (+30°)...")
        print("👀 Watch the front wheels - they should turn RIGHT")
        car.set_akm_steering_angle(30)
        time.sleep(2)
        print("✅ Right turn complete")

        # Test 6: Back to center
        print("\n[TEST 6] Returning to CENTER (0°)...")
        car.set_akm_steering_angle(0)
        time.sleep(2)
        print("✅ Final center position")

        # Test 7: Full range sweep
        print("\n[TEST 7] Sweeping full steering range (-45° to +45°)...")
        print("👀 Watch the wheels sweep left to right")

        for angle in range(-45, 46, 5):
            car.set_akm_steering_angle(angle)
            time.sleep(0.2)

        # Return to center
        car.set_akm_steering_angle(0)
        print("✅ Sweep complete")

        print("\n" + "="*60)
        print("  STEERING TEST COMPLETE!")
        print("="*60)
        print("\n✅ If front wheels moved, steering servo is working!")
        print("\nℹ️  Steering angle range: -45° (left) to +45° (right)")
        print("ℹ️  Default angle can be adjusted with:")
        print("    car.set_akm_default_angle(angle, forever=False)")
        print("\n❌ If wheels didn't move:")
        print("   - Check servo power connection")
        print("   - Check servo cable connection")
        print("   - Verify this is an R2 (Ackermann) robot")

    except Exception as e:
        print(f"\n❌ Error: {e}")
        # Try to return to center
        try:
            car.set_akm_steering_angle(0)
        except:
            pass
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        # Disable auto-report and cleanup
        car.set_auto_report_state(False, forever=False)

if __name__ == "__main__":
    main()
