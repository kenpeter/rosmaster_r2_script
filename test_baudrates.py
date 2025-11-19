#!/usr/bin/env python3
"""
Test different baudrates to find the correct one for YDLidar X4
"""
import serial
import time

# Common YDLidar baudrates
baudrates = [115200, 128000, 153600, 230400, 512000]

port = '/dev/ydlidar'

for baud in baudrates:
    print(f"\n{'='*60}")
    print(f"Testing baudrate: {baud}")
    print('='*60)

    try:
        ser = serial.Serial(port, baud, timeout=0.5)

        # Enable motor
        ser.setDTR(True)
        time.sleep(0.3)

        # Clear any existing data
        ser.reset_input_buffer()

        # Send GET_INFO command
        print("Sending GET_INFO (A5 90)...")
        ser.write(bytes([0xA5, 0x90]))
        ser.flush()
        time.sleep(0.2)

        if ser.in_waiting > 0:
            response = ser.read(min(100, ser.in_waiting))
            print(f"Response ({len(response)} bytes):")

            # Check for valid response header (A5 5A)
            if len(response) >= 2 and response[0] == 0xA5 and response[1] == 0x5A:
                print("✓ VALID RESPONSE HEADER FOUND! (A5 5A)")
                print(f"  Full response: {response.hex(' ')}")

                # Decode device info if possible
                if len(response) >= 20:
                    print(f"  Model: {response[6]}")
                    print(f"  Firmware: {response[7]}.{response[8]}")
                    print(f"  Hardware: {response[9]}")
                    print(f"  Serial: {response[10:18].hex()}")

            else:
                print(f"✗ Invalid/corrupted response: {response[:20].hex(' ')}...")

        else:
            print("✗ No response")

        # Cleanup
        ser.setDTR(False)
        ser.close()

    except Exception as e:
        print(f"✗ Error: {e}")

print(f"\n{'='*60}")
print("Testing complete!")
