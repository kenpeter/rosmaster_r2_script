#!/usr/bin/env python3
# Reset and recover ROS Master board communication

import sys
import time
import serial

print("=" * 70)
print("  ROS Master Board Reset & Recovery Tool")
print("=" * 70)
print()

port = "/dev/ttyUSB1"

print("This tool will attempt to reset the ROS Master board communication.")
print(f"Target port: {port}")
print()
print("Steps:")
print("  1. Close the serial port completely")
print("  2. Wait for device to reset")
print("  3. Reopen with proper initialization")
print("  4. Send reset command")
print("  5. Verify communication")
print()

input("Press ENTER to continue (or Ctrl+C to abort)...")

# Step 1: Try to close any existing connections
print("\n" + "=" * 70)
print("Step 1: Closing existing serial connections...")
print("=" * 70)

try:
    # Import after sys.path modification
    sys.path.append('/home/jetson/yahboomcar_ros2_ws/software/py_install_V3.3.1')
    import serial.tools.list_ports

    # List all USB serial ports
    ports = serial.tools.list_ports.comports()
    print(f"Found {len(ports)} serial ports:")
    for p in ports:
        print(f"  - {p.device}: {p.description}")
except Exception as e:
    print(f"Note: {e}")

print()

# Step 2: Check if port is accessible
print("=" * 70)
print("Step 2: Checking port accessibility...")
print("=" * 70)

try:
    import os
    import stat

    if os.path.exists(port):
        st = os.stat(port)
        print(f"✅ Port exists: {port}")
        print(f"   Permissions: {oct(st.st_mode)[-3:]}")

        # Check if we can access it
        if os.access(port, os.R_OK | os.W_OK):
            print(f"✅ Port is readable and writable")
        else:
            print(f"❌ Port is not accessible - trying chmod...")
            import subprocess
            subprocess.run(['sudo', 'chmod', '666', port], check=False)
            time.sleep(0.5)
    else:
        print(f"❌ Port does not exist: {port}")
        print("   Please check USB connection!")
        sys.exit(1)
except Exception as e:
    print(f"⚠️  Error checking port: {e}")

print()

# Step 3: Power cycle detection
print("=" * 70)
print("Step 3: Power Cycle Recommendation")
print("=" * 70)
print()
print("⚠️  IMPORTANT: The ROS Master board may need a power cycle.")
print()
print("Option A - Full Power Cycle (RECOMMENDED):")
print("  1. Disconnect the ROS Master board power cable")
print("  2. Wait 10 seconds")
print("  3. Reconnect the power cable")
print("  4. Wait for the board to boot (look for LED activity)")
print("  5. Run this script again")
print()
print("Option B - USB Reset (Try this first):")
print("  1. Unplug the USB cable from the Jetson")
print("  2. Wait 5 seconds")
print("  3. Plug it back in")
print("  4. Continue with this script")
print()

choice = input("Have you done a power cycle? (y/n) or press 'u' to try USB reset: ").lower()

if choice == 'u':
    print("\nAttempting USB reset...")
    print("Please unplug the USB cable NOW...")
    input("Press ENTER after unplugging...")
    print("Waiting 5 seconds...")
    time.sleep(5)
    print("Please plug the USB cable back in NOW...")
    input("Press ENTER after plugging back in...")
    print("Waiting for device to enumerate...")
    time.sleep(3)
elif choice != 'y':
    print("\nPlease power cycle the board and run this script again.")
    sys.exit(0)

# Step 4: Try to communicate
print()
print("=" * 70)
print("Step 4: Attempting communication...")
print("=" * 70)

try:
    from Rosmaster_Lib import Rosmaster

    print("Opening serial port with Rosmaster library...")
    bot = Rosmaster(car_type=5, com=port, debug=True)

    print("\nCreating receive thread...")
    bot.create_receive_threading()
    time.sleep(1)

    print("\nSending reset state command...")
    # FUNC_RESET_STATE = 0x0F
    HEAD = 0xFF
    DEVICE_ID = 0xFC
    FUNC_RESET_STATE = 0x0F
    COMPLEMENT = 257 - DEVICE_ID

    cmd = [HEAD, DEVICE_ID, 0x03, FUNC_RESET_STATE]
    checksum = (sum(cmd) + COMPLEMENT) & 0xff
    cmd.append(checksum)

    bot.ser.write(bytes(cmd))
    print(f"Sent reset command: {bytes(cmd).hex()}")
    time.sleep(2)

    print("\nSetting car type to R2...")
    bot.set_car_type(5)
    time.sleep(0.5)

    print("\nEnabling auto-report...")
    bot.set_auto_report_state(True, False)
    time.sleep(0.5)

    print("\nTesting beep...")
    bot.set_beep(100)
    time.sleep(0.5)

    print("\nReading version...")
    for attempt in range(3):
        version = bot.get_version()
        print(f"  Attempt {attempt + 1}: Version = {version}")
        if version > 0:
            print(f"✅ SUCCESS! Firmware version: {version}")
            break
        time.sleep(0.5)

    print("\nReading battery voltage...")
    for attempt in range(3):
        voltage = bot.get_battery_voltage()
        print(f"  Attempt {attempt + 1}: Voltage = {voltage}V")
        if voltage > 0:
            print(f"✅ SUCCESS! Battery voltage: {voltage}V")
            break
        time.sleep(0.5)

    print("\n" + "=" * 70)
    print("  Recovery Attempt Complete")
    print("=" * 70)

    if version > 0 or voltage > 0:
        print("\n✅ Communication appears to be working!")
        print("\nNext steps:")
        print("  1. Try running: ./test_motors_direct.py")
        print("  2. If that works, restart ROS: ./start_robot.sh")
    else:
        print("\n❌ Communication still not working")
        print("\nThis indicates a hardware issue:")
        print("  1. ROS Master board may be defective")
        print("  2. Firmware may be corrupted")
        print("  3. Power supply may be insufficient")
        print("  4. USB cable may be faulty")
        print()
        print("Recommended:")
        print("  - Try a different USB cable")
        print("  - Check if the board has any visible damage")
        print("  - Contact Yahboom support")

except Exception as e:
    print(f"\n❌ Error: {e}")
    import traceback
    traceback.print_exc()
    print("\nThe board may need firmware reflashing or hardware repair.")

print()
