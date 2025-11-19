#!/usr/bin/env python3
"""
Debug voltage reading from Rosmaster library
"""

import sys
sys.path.append('/home/jetson/yahboomcar_ros2_ws/software/py_install_V3.3.1')

from Rosmaster_Lib import Rosmaster
import time

print("\n" + "="*60)
print("  DEBUG VOLTAGE READING")
print("="*60 + "\n")

car = Rosmaster()
car.create_receive_threading()

print("Waiting for data...")
time.sleep(2)

for i in range(10):
    voltage = car.get_battery_voltage()
    print(f"Reading {i+1}: {voltage:.2f}V")
    time.sleep(0.5)

print("\n" + "="*60)
print("If all readings show 0.00V, the issue is in the")
print("Rosmaster_Lib parsing or the packet structure.")
print("="*60)
