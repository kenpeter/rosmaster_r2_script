#!/usr/bin/env python3
"""
Test YDLidar X4 INTERNAL motor (the spinning laser scanner)
NOT the robot wheel motors!
"""
import serial
import time

port = '/dev/ydlidar'
baudrate = 128000

print("="*60)
print("YDLidar X4 INTERNAL Motor Test")
print("="*60)
print("")
print("This test controls the LIDAR's internal motor that makes")
print("the laser scanner spin around (360 degrees).")
print("")
print("You should:")
print("  - HEAR: Quiet whirring/humming from the lidar")
print("  - FEEL: Gentle vibration if you touch the lidar")
print("  - SEE: The black top part of the lidar spinning")
print("")
print("="*60)
print("")

print("Opening serial port...")
ser = serial.Serial(port, baudrate, timeout=1)

print("\n--- TEST 1: Motor OFF for 3 seconds ---")
print("The lidar motor should be STOPPED/SILENT")
ser.setDTR(False)
time.sleep(3)

print("\n--- TEST 2: Motor ON for 5 seconds ---")
print("*** TOUCH THE LIDAR NOW ***")
print("The lidar motor should be SPINNING!")
print("You should hear/feel it!")
ser.setDTR(True)
time.sleep(5)

print("\n--- TEST 3: Motor OFF for 3 seconds ---")
print("Motor should STOP again")
ser.setDTR(False)
time.sleep(3)

print("\n--- TEST 4: Motor ON again for 5 seconds ---")
print("*** TOUCH THE LIDAR AGAIN ***")
print("Motor should spin again!")
ser.setDTR(True)
time.sleep(5)

print("\n--- Final: Motor OFF ---")
ser.setDTR(False)
ser.close()

print("\n" + "="*60)
print("Test complete!")
print("="*60)
print("\nDid the LIDAR's internal motor spin?")
print("(NOT the robot wheels - the lidar's own laser motor)")
print("")
print("If NO:")
print("  - Check PWR MicroUSB cable is firmly connected")
print("  - Check USB hub has power adapter plugged in")
print("  - The lidar motor might be broken")
print("="*60)
