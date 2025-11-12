#!/usr/bin/env python3
# Direct motor control test using Rosmaster_Lib
# This mimics what the ROS driver does

import sys
import time
sys.path.append('/home/jetson/yahboomcar_ros2_ws/software/py_install_V3.3.1')

from Rosmaster_Lib import Rosmaster

def main():
    print("=" * 60)
    print("  Direct Motor Control Test using Rosmaster_Lib")
    print("=" * 60)
    print()

    # Initialize Rosmaster (same as driver does)
    print("Step 1: Initializing Rosmaster...")
    try:
        bot = Rosmaster(car_type=5, com="/dev/ttyUSB1", debug=False)
        print("✅ Rosmaster initialized")
    except Exception as e:
        print(f"❌ Failed to initialize Rosmaster: {e}")
        return False

    # Create receive thread (critical!)
    print("\nStep 2: Creating receive thread...")
    bot.create_receive_threading()
    time.sleep(0.5)  # Give thread time to start
    print("✅ Receive thread created")

    # Set car type
    print("\nStep 3: Setting car type to R2 (5)...")
    bot.set_car_type(5)
    time.sleep(0.2)
    print("✅ Car type set")

    # Get version
    print("\nStep 4: Getting firmware version...")
    version = bot.get_version()
    print(f"   Firmware version: {version}")
    if version < 0:
        print("⚠️  WARNING: Invalid version number!")
        print("   This usually means:")
        print("   - Board needs power cycle")
        print("   - Communication sync issue")
    else:
        print(f"✅ Valid firmware version: {version}")

    # Get battery voltage
    print("\nStep 5: Getting battery voltage...")
    voltage = bot.get_battery_voltage()
    print(f"   Battery voltage: {voltage}V")
    if voltage == 0:
        print("⚠️  WARNING: Zero voltage reading!")
        print("   - Check if battery is connected")
        print("   - Check if board is powered")
    else:
        print(f"✅ Battery voltage: {voltage}V")

    # Test beep
    print("\nStep 6: Testing beep (you should hear a sound)...")
    bot.set_beep(100)  # Beep for 100ms
    time.sleep(0.5)
    print("   Did you hear a beep?")

    # Read current motion data
    print("\nStep 7: Reading current motion data...")
    vx, vy, vz = bot.get_motion_data()
    print(f"   Current motion: vx={vx}, vy={vy}, vz={vz}")

    # Test motor control
    print("\n" + "=" * 60)
    print("  MOTOR MOVEMENT TEST")
    print("=" * 60)
    input("\n⚠️  Press ENTER to test motor movement (or Ctrl+C to abort)...")

    print("\nTest 1: FORWARD at 0.3 m/s for 3 seconds...")
    start_time = time.time()
    count = 0
    while time.time() - start_time < 3.0:
        bot.set_car_motion(0.3, 0, 0)
        count += 1
        time.sleep(0.05)  # 20Hz update rate

    print(f"   Sent {count} forward commands")

    # Stop
    print("   STOPPING...")
    bot.set_car_motion(0, 0, 0)
    time.sleep(0.5)

    print("\nTest 2: ROTATE LEFT at 0.5 rad/s for 2 seconds...")
    start_time = time.time()
    count = 0
    while time.time() - start_time < 2.0:
        bot.set_car_motion(0, 0, 0.5)
        count += 1
        time.sleep(0.05)

    print(f"   Sent {count} rotation commands")

    # Stop
    print("   STOPPING...")
    bot.set_car_motion(0, 0, 0)
    time.sleep(0.5)

    # Check motion feedback
    print("\nStep 8: Reading motion feedback...")
    vx, vy, vz = bot.get_motion_data()
    print(f"   Final motion data: vx={vx}, vy={vy}, vz={vz}")

    print("\n" + "=" * 60)
    print("  Test Complete!")
    print("=" * 60)
    print()
    print("Results:")
    print(f"  - Firmware version: {version}")
    print(f"  - Battery voltage: {voltage}V")
    print(f"  - Motor commands sent: {count * 2} total")
    print()

    if version > 0 and voltage > 0:
        print("✅ Communication with ROS Master board is WORKING")
        print()
        print("If motors DID NOT move, check:")
        print("  1. Are motors powered? (Check power LED on motor driver)")
        print("  2. Are motor connectors plugged in correctly?")
        print("  3. Is battery charged? (Needs >7V typically)")
        print("  4. Are there any error LEDs on the board?")
    else:
        print("❌ Communication issue detected")
        print()
        print("Recommended actions:")
        print("  1. Power cycle the ROS Master board")
        print("  2. Check USB connection")
        print("  3. Try: sudo chmod 666 /dev/ttyUSB*")

    return True

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\n\n❌ Error during test: {e}")
        import traceback
        traceback.print_exc()
