#!/usr/bin/env python3
# Find which USB port has the ROS Master board

import sys
import time
sys.path.append('/home/jetson/yahboomcar_ros2_ws/software/py_install_V3.3.1')

from Rosmaster_Lib import Rosmaster

ports_to_test = ["/dev/ttyUSB0", "/dev/ttyUSB1"]

print("=" * 70)
print("  Finding ROS Master Board")
print("=" * 70)
print()
print("Testing ports:", ports_to_test)
print()

for port in ports_to_test:
    print("=" * 70)
    print(f"Testing: {port}")
    print("=" * 70)

    try:
        print(f"  Opening {port}...")
        bot = Rosmaster(car_type=5, com=port, debug=False)

        print(f"  Creating receive thread...")
        bot.create_receive_threading()
        time.sleep(1)

        print(f"  Setting car type...")
        bot.set_car_type(5)
        time.sleep(0.3)

        print(f"  Requesting version...")
        version = bot.get_version()
        time.sleep(0.3)

        print(f"  Requesting voltage...")
        voltage = bot.get_battery_voltage()
        time.sleep(0.3)

        print(f"\n  Results:")
        print(f"    Version: {version}")
        print(f"    Voltage: {voltage}V")

        if version > 0:
            print(f"\n✅ FOUND ROS MASTER on {port}!")
            print(f"   Firmware version: {version}")
            print(f"   Battery voltage: {voltage}V")
            print()
            print(f"🔧 Action needed:")
            print(f"   Update /dev/myserial symlink to point to {port}")
            print(f"   Command: sudo ln -sf {port.split('/')[-1]} /dev/myserial")
            sys.exit(0)
        elif voltage > 0:
            print(f"\n⚠️  Partial response on {port}")
            print(f"   Voltage reading works but version failed")
            print(f"   This might be the ROS Master board")
        else:
            print(f"\n❌ Not the ROS Master (no valid response)")

        # Close the port
        bot.ser.close()
        time.sleep(0.5)

    except Exception as e:
        print(f"\n❌ Error testing {port}: {e}")

    print()

print("=" * 70)
print("  Search Complete")
print("=" * 70)
print()
print("❌ ROS Master board not found on any port!")
print()
print("Possible issues:")
print("  1. ROS Master board is not powered on")
print("  2. USB cable is disconnected or faulty")
print("  3. Board firmware has crashed/corrupted")
print("  4. Board hardware failure")
print()
print("Try:")
print("  1. Power cycle the ROS Master board")
print("  2. Check all cable connections")
print("  3. Look for LED indicators on the board")
print()
