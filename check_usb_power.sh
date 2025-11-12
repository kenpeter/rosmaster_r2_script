#!/bin/bash
# Check USB power and low-level device information

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}======================================================================${NC}"
echo -e "${BLUE}  USB Power and Device Analysis${NC}"
echo -e "${BLUE}======================================================================${NC}"
echo ""

# Find the RPLiDAR device
VENDOR_ID="10c4"
PRODUCT_ID="ea60"

echo -e "${YELLOW}[1/7] Locating RPLiDAR device...${NC}"
DEVICE_PATH=$(lsusb -d ${VENDOR_ID}:${PRODUCT_ID} | head -1)
if [ -z "$DEVICE_PATH" ]; then
    echo -e "${RED}❌ RPLiDAR not found (VID:PID ${VENDOR_ID}:${PRODUCT_ID})${NC}"
    echo ""
    echo "All USB devices:"
    lsusb
    exit 1
fi

echo -e "${GREEN}✅ Found: $DEVICE_PATH${NC}"
echo ""

# Extract bus and device number
BUS=$(echo "$DEVICE_PATH" | awk '{print $2}')
DEV=$(echo "$DEVICE_PATH" | awk '{print $4}' | tr -d ':')

echo -e "${YELLOW}[2/7] USB Location:${NC}"
echo "   Bus: $BUS"
echo "   Device: $DEV"
echo ""

# Get detailed device info
echo -e "${YELLOW}[3/7] Detailed Device Information:${NC}"
sudo lsusb -v -s ${BUS}:${DEV} 2>/dev/null | head -60
echo ""

# Check power consumption
echo -e "${YELLOW}[4/7] Power Information:${NC}"
MAX_POWER=$(sudo lsusb -v -s ${BUS}:${DEV} 2>/dev/null | grep "MaxPower" | awk '{print $2}')
echo "   MaxPower: $MAX_POWER"

if [ ! -z "$MAX_POWER" ]; then
    POWER_MA=$(echo $MAX_POWER | tr -d 'mA')
    if [ $POWER_MA -lt 500 ]; then
        echo -e "${GREEN}   ✅ Power requirement OK (<500mA)${NC}"
    else
        echo -e "${YELLOW}   ⚠️  High power requirement (${POWER_MA}mA)${NC}"
        echo "      May need powered USB hub"
    fi
fi
echo ""

# Check USB speed
echo -e "${YELLOW}[5/7] USB Speed:${NC}"
SPEED=$(sudo lsusb -v -s ${BUS}:${DEV} 2>/dev/null | grep "bcdUSB" | head -1)
echo "   $SPEED"
echo ""

# Check kernel messages
echo -e "${YELLOW}[6/7] Recent Kernel Messages (last 20 lines):${NC}"
sudo dmesg | grep -i "ttyUSB\|cp210x\|usb.*${BUS}-" | tail -20
echo ""

# Check sysfs for power info
echo -e "${YELLOW}[7/7] System Power Status:${NC}"
SYS_PATH="/sys/bus/usb/devices/${BUS}-*"

for device in $SYS_PATH; do
    if [ -d "$device" ]; then
        echo "   Device: $device"

        if [ -f "$device/product" ]; then
            PRODUCT=$(cat "$device/product" 2>/dev/null)
            if [[ "$PRODUCT" == *"CP210"* ]] || [[ "$PRODUCT" == *"UART"* ]]; then
                echo "   Product: $PRODUCT"

                if [ -f "$device/power/level" ]; then
                    echo "   Power Level: $(cat $device/power/level)"
                fi

                if [ -f "$device/power/control" ]; then
                    echo "   Power Control: $(cat $device/power/control)"
                fi

                if [ -f "$device/authorized" ]; then
                    echo "   Authorized: $(cat $device/authorized)"
                fi

                echo ""
            fi
        fi
    fi
done

echo -e "${BLUE}======================================================================${NC}"
echo -e "${BLUE}  Recommendations${NC}"
echo -e "${BLUE}======================================================================${NC}"
echo ""
echo "1. If MaxPower > 400mA:"
echo "   - Try a different USB port"
echo "   - Use a powered USB 3.0 hub"
echo ""
echo "2. Check physical connection:"
echo "   - Unplug and replug the RPLiDAR"
echo "   - Try a different USB cable"
echo ""
echo "3. Monitor kernel messages while plugging in:"
echo "   sudo dmesg -w"
echo "   (then plug/unplug RPLiDAR to see messages)"
echo ""
