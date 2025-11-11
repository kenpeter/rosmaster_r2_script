import serial
import time

print("Connecting to LiDAR data port...")
ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=2)

print("Setting DTR/RTS...")
ser.setDTR(False)
ser.setRTS(False)
time.sleep(0.1)

print("Sending STOP command first...")
ser.write(bytes([0xA5, 0x25]))
time.sleep(0.5)

print("Sending motor START command (max speed)...")
# Start motor: 0xA5 0xF0 0x02 [PWM_LSB] [PWM_MSB] [Checksum]
# Max PWM = 1023 (0x3FF) = 0xFF 0x03
ser.write(bytes([0xA5, 0xF0, 0x02, 0xFF, 0x03, 0x00]))
time.sleep(2)

print("\n🎯 CHECK NOW: Is the motor spinning?")
print("You should hear/see it rotate!")
time.sleep(3)

ser.close()
print("Test complete")

