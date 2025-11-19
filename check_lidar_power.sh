#!/bin/bash
# Check YDLidar power information

echo "========================================"
echo "YDLidar X4 Power Status Check"
echo "========================================"
echo ""

# Find the lidar USB device
LIDAR_DEV="/sys/devices/platform/bus@0/3610000.usb/usb1/1-2/1-2.4/1-2.4.3"

echo "USB Device Info:"
echo "  Vendor:  $(cat $LIDAR_DEV/manufacturer 2>/dev/null || echo 'Unknown')"
echo "  Product: $(cat $LIDAR_DEV/product 2>/dev/null || echo 'Unknown')"
echo "  Vendor ID: $(cat $LIDAR_DEV/idVendor)"
echo "  Product ID: $(cat $LIDAR_DEV/idProduct)"
echo ""

echo "Power Configuration:"
echo "  Max Power Draw: $(cat $LIDAR_DEV/bMaxPower) (reported by device)"
echo "  USB Speed: $(cat $LIDAR_DEV/speed) Mbps"
echo ""

echo "YDLidar X4 Requirements (from specs):"
echo "  Voltage: 5V (4.8V - 5.2V)"
echo "  Current (idle): ~100mA"
echo "  Current (motor running): ~400-500mA"
echo "  Total power needed: ~2.5W"
echo ""

echo "========================================"
echo "IMPORTANT:"
echo "========================================"
echo "The YDLidar X4 USB adapter has TWO ports:"
echo "  1. DATA port (connects to Jetson) - for serial communication"
echo "  2. PWR port (MicroUSB) - for auxiliary 5V power"
echo ""
echo "If your lidar motor doesn't spin, you MUST connect"
echo "a 5V power supply to the PWR MicroUSB port!"
echo ""
echo "Check your lidar adapter board for a port labeled 'PWR'"
echo "========================================"
