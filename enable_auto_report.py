#!/usr/bin/env python3
"""
Enable auto-report on Rosmaster board
This makes the board send voltage/speed data automatically
"""

import sys
sys.path.append('/home/jetson/yahboomcar_ros2_ws/software/py_install_V3.3.1')

from Rosmaster_Lib import Rosmaster
import time

print("\n" + "="*60)
print("  ENABLE ROSMASTER AUTO-REPORT")
print("="*60 + "\n")

car = Rosmaster()
print("Enabling auto-report (forever=True)...")
car.set_auto_report_state(enable=True, forever=True)

time.sleep(2)

print("Auto-report enabled! The board should now send:")
print("  - Voltage data")
print("  - Speed data")
print("  - IMU data")
print("  - Encoder data")
print("\nChecking voltage...")

car.create_receive_threading()
time.sleep(2)

for i in range(5):
    voltage = car.get_battery_voltage()
    print(f"  Reading {i+1}: {voltage:.2f}V")
    time.sleep(0.5)

print("\n" + "="*60)
print("If voltage now shows 12.2V, the fix worked!")
print("Restart the ROS driver to see /voltage topic update.")
print("="*60)
