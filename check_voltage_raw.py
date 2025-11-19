#!/usr/bin/env python3
"""
Check raw voltage data from Rosmaster board
"""

import serial
import struct
import time

print("\n" + "="*60)
print("  RAW VOLTAGE DATA DIAGNOSTIC")
print("="*60 + "\n")

# Find motor board
port = '/dev/ttyUSB0'
try:
    ser = serial.Serial(port, 115200, timeout=2)
    print(f"✅ Connected to Rosmaster board: {port}\n")

    time.sleep(0.5)
    ser.reset_input_buffer()
    time.sleep(0.2)

    print("📡 Reading data packets...\n")

    voltage_found = False
    for attempt in range(10):
        data = ser.read(500)

        if len(data) > 0:
            print(f"Received {len(data)} bytes")

        # Find ALL packets and show what we're getting
        for i in range(len(data)-4):
            if data[i] == 0xff and data[i+1] == 0xfb:
                # Found packet start
                if i+3 < len(data):
                    packet_len = data[i+2]
                    func_type = data[i+3]

                    print(f"\nFound packet type: 0x{func_type:02x}")

                    # Show first 20 bytes of packet
                    packet_end = min(i+20, len(data))
                    packet_bytes = ' '.join([f'{b:02x}' for b in data[i:packet_end]])
                    print(f"  Bytes: {packet_bytes}")

                    # FUNC_REPORT_SPEED = 0x0A contains voltage at byte 6
                    if func_type == 0x0a and i+10 < len(data):
                        # Voltage is at ext_data[6], which is data[i+4+6] = data[i+10]
                        voltage_raw = data[i+10]
                        voltage_float = voltage_raw / 10.0

                        print(f"\n  ✅ VOLTAGE PACKET FOUND!")
                        print(f"  Raw voltage byte: {voltage_raw} (0x{voltage_raw:02x})")
                        print(f"  Calculated voltage: {voltage_float:.2f}V")
                        voltage_found = True

        time.sleep(0.3)

    if not voltage_found:
        print("\n⚠️  WARNING: No voltage packets (type 0x0A) found!")
        print("   The Rosmaster board may not be sending speed/voltage data.")

    ser.close()

    print("\n" + "="*60)
    print("  ANALYSIS")
    print("="*60 + "\n")

    print("If raw voltage byte is 0 or 1:")
    print("  → Voltage sensing circuit is not working")
    print("  → Possible causes:")
    print("    • Voltage divider resistors issue")
    print("    • ADC pin not connected")
    print("    • Firmware issue\n")

    print("If raw voltage byte is 110-126:")
    print("  → Normal (11.0V - 12.6V battery)")
    print("  → Voltage sensor is working correctly\n")

    print("If raw voltage byte is changing:")
    print("  → Sensor is working, just reading low voltage")
    print("  → Battery needs more charging\n")

except Exception as e:
    print(f"❌ Error: {e}")
    import traceback
    traceback.print_exc()
