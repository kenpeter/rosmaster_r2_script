#!/bin/bash

echo "========================================="
echo "  Yahboom Robot Hardware Diagnostics"
echo "========================================="
echo ""

# Check if ch34x driver is loaded
echo "1. Checking USB serial driver..."
if lsmod | grep -q ch34x; then
    echo "   ✅ ch34x driver is loaded"
else
    echo "   ⚠️  ch34x driver not loaded - attempting to load..."
    sudo modprobe ch34x
    sleep 1
    if lsmod | grep -q ch34x; then
        echo "   ✅ ch34x driver loaded successfully"
    else
        echo "   ❌ Failed to load ch34x driver"
    fi
fi
echo ""

# Check USB devices
echo "2. Checking USB devices..."
if lsusb | grep -q "1a86:7522"; then
    echo "   ✅ Robot controller USB detected (1a86:7522)"
elif lsusb | grep -q "1a86:7523"; then
    echo "   ✅ Robot controller USB detected (1a86:7523)"
else
    echo "   ❌ Robot controller USB NOT detected"
    echo "      Please check USB connection"
fi

if lsusb | grep -q "10c4:ea60"; then
    echo "   ✅ LiDAR USB detected (CP2102)"
elif lsusb | grep -q "Silicon Labs"; then
    echo "   ✅ LiDAR USB detected (Silicon Labs)"
else
    echo "   ⚠️  LiDAR USB NOT detected"
fi

if lsusb | grep -q "Orbbec"; then
    echo "   ✅ Camera USB detected (Orbbec)"
else
    echo "   ⚠️  Camera USB NOT detected"
fi
echo ""

# Check device files
echo "3. Checking device files..."
if [ -e /dev/ttyUSB0 ]; then
    echo "   ✅ /dev/ttyUSB0 exists"
    ls -la /dev/ttyUSB0
else
    echo "   ❌ /dev/ttyUSB0 does NOT exist"
fi

if [ -e /dev/ttyUSB1 ]; then
    echo "   ✅ /dev/ttyUSB1 exists"
    ls -la /dev/ttyUSB1
else
    echo "   ⚠️  /dev/ttyUSB1 does NOT exist"
fi

if [ -e /dev/myserial ]; then
    echo "   ✅ /dev/myserial exists"
    ls -la /dev/myserial
else
    echo "   ❌ /dev/myserial does NOT exist"
    echo "      This is the robot controller serial port"
fi
echo ""

# Check udev rules
echo "4. Checking udev rules..."
if [ -f /etc/udev/rules.d/serial.rules ]; then
    echo "   ✅ serial.rules exists"
    cat /etc/udev/rules.d/serial.rules | grep -v "^$"
else
    echo "   ❌ serial.rules NOT found"
fi
echo ""

echo "========================================="
echo "  Recommendations:"
echo "========================================="

if ! lsmod | grep -q ch34x; then
    echo "❌ Load ch34x driver: sudo modprobe ch34x"
fi

if ! [ -e /dev/ttyUSB0 ] && lsusb | grep -q "1a86:75"; then
    echo "⚠️  USB device detected but no /dev/ttyUSB* created"
    echo "   Try: sudo udevadm control --reload-rules && sudo udevadm trigger"
    echo "   Or unplug and replug the USB cable"
fi

if ! [ -e /dev/myserial ] && [ -e /dev/ttyUSB0 ]; then
    echo "⚠️  ttyUSB0 exists but myserial symlink not created"
    echo "   Check udev rules and try: sudo udevadm trigger"
fi

echo ""
echo "Done!"
