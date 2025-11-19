#!/bin/bash
# Complete USB power scan for YDLidar debugging

echo "========================================"
echo "Complete USB Power Scan Report"
echo "========================================"
echo ""

echo "=== Serial Devices (CH340/CH341 chips) ==="
echo ""

# YDLidar (ttyUSB2)
echo "1. YDLidar X4 (/dev/ttyUSB2):"
echo "   Vendor: 1a86 (QinHeng Electronics)"
echo "   Product: 7522 (CH341 USB-Serial)"
echo "   Max Power: 98mA"
echo "   Status: ⚠️  ONLY ENOUGH FOR SERIAL CHIP!"
echo "   Note: Motor needs 400-500mA additional power"
echo ""

# Rosmaster (ttyUSB0)
echo "2. Rosmaster Board (/dev/ttyUSB0):"
echo "   Vendor: 1a86 (QinHeng Electronics)"
echo "   Product: 7523 (CH340 USB-Serial)"
echo "   Max Power: 104mA"
echo "   Status: ✓ OK (no motor, just communication)"
echo ""

# ToF sensor (ttyUSB1)
echo "3. ToF Depth Sensor (/dev/ttyUSB1):"
echo "   Vendor: 10c4 (Silicon Labs)"
echo "   Product: ea60 (CP2102 UART Bridge)"
echo "   Max Power: 100mA"
echo "   Status: ✓ OK"
echo ""

echo "=== Other Devices ==="
echo ""
echo "Orbbec Camera: 500mA + 500mA = 1000mA total"
echo "Razer Mouse: 500mA"
echo "Bluetooth: 100mA"
echo ""

echo "========================================"
echo "POWER ISSUE IDENTIFIED:"
echo "========================================"
echo ""
echo "YDLidar X4 Power Budget:"
echo "  - USB Serial Chip: 98mA (provided by DATA port)"
echo "  - Motor: 400-500mA (NOT PROVIDED!)"
echo "  - Sensor: 50-100mA (NOT PROVIDED!)"
echo "  - TOTAL NEEDED: ~600mA"
echo "  - CURRENTLY PROVIDED: 98mA ❌"
echo ""
echo "SOLUTION:"
echo "  The YDLidar X4 has a separate PWR MicroUSB port"
echo "  that must be connected to 5V power supply!"
echo ""
echo "  Look for a second MicroUSB port on the lidar"
echo "  adapter board labeled 'PWR' or 'POWER'"
echo "========================================"
