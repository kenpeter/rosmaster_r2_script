#!/bin/bash
# RPLiDAR Power and Connection Test Script

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}======================================================================${NC}"
echo -e "${BLUE}  RPLiDAR Power Cycling and Connection Test${NC}"
echo -e "${BLUE}======================================================================${NC}"
echo ""

echo -e "${YELLOW}This script will help test if the RPLiDAR has a power issue.${NC}"
echo ""
echo "We will:"
echo "  1. Check current USB connection"
echo "  2. Power cycle the RPLiDAR"
echo "  3. Test on different USB ports"
echo ""

# Check current status
echo -e "${BLUE}[1/5] Current RPLiDAR Status${NC}"
if lsusb | grep -q "10c4:ea60"; then
    echo -e "${GREEN}✅ RPLiDAR USB device detected${NC}"
    lsusb | grep "10c4:ea60"
else
    echo -e "${RED}❌ RPLiDAR not detected!${NC}"
    echo "Please ensure it's plugged in."
    exit 1
fi
echo ""

# Check power
echo -e "${BLUE}[2/5] Checking USB Power Configuration${NC}"
POWER=$(lsusb -v -d 10c4:ea60 2>/dev/null | grep "MaxPower" | awk '{print $2}')
echo "   Declared MaxPower: ${POWER}"

if [ "$POWER" = "100mA" ]; then
    echo -e "${YELLOW}   ⚠️  WARNING: Device declares only 100mA${NC}"
    echo "      RPLiDAR motor typically needs 400-500mA!"
    echo "      This may indicate:"
    echo "        - Wrong firmware/configuration"
    echo "        - Device is not genuine RPLiDAR A1"
    echo "        - Power delivery issue"
else
    echo -e "${GREEN}   ✅ Power declaration seems reasonable${NC}"
fi
echo ""

# Suggest power cycle
echo -e "${BLUE}[3/5] Power Cycle Test${NC}"
echo "Please perform these steps:"
echo ""
echo "  1. Unplug the RPLiDAR USB cable"
echo "  2. Wait 5 seconds"
echo "  3. Listen carefully as you plug it back in"
echo "  4. You should hear a brief motor sound when plugged in"
echo ""
echo -e "${YELLOW}Press Enter after you've plugged it back in...${NC}"
read

# Check if it reconnected
sleep 2
if lsusb | grep -q "10c4:ea60"; then
    echo -e "${GREEN}✅ RPLiDAR reconnected${NC}"

    # Check which ttyUSB it's on
    sleep 1
    if [ -e "/dev/rplidar" ]; then
        TARGET=$(readlink -f /dev/rplidar)
        echo "   Device: /dev/rplidar -> $TARGET"
    fi
else
    echo -e "${RED}❌ RPLiDAR did not reconnect!${NC}"
    echo "   This suggests a hardware problem."
    exit 1
fi
echo ""

# Test DTR control
echo -e "${BLUE}[4/5] Testing Motor Control (DTR signal)${NC}"
echo "Running Python diagnostic..."
python3 /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/scripts/rplidar_diagnose.py 2>&1 | grep -E "Motor Control|Received:|ERROR|motor"
echo ""

# Final recommendations
echo -e "${BLUE}[5/5] Recommendations${NC}"
echo ""
echo "Based on the diagnostics, here are the next steps:"
echo ""
echo "If motor still doesn't spin:"
echo ""
echo "  Option A: Try Different USB Port"
echo "    1. Unplug RPLiDAR"
echo "    2. Plug into a different USB 3.0 port (blue port if available)"
echo "    3. Run this test again"
echo ""
echo "  Option B: Use Powered USB Hub"
echo "    1. Get a powered USB 3.0 hub with its own power supply"
echo "    2. Connect RPLiDAR through the hub"
echo "    3. This provides more stable power"
echo ""
echo "  Option C: Contact Yahboom Support"
echo "    The RPLiDAR may be defective and need replacement"
echo "    - MaxPower of 100mA is unusually low"
echo "    - No response to any commands indicates hardware failure"
echo ""
echo "  Option D: Run Robot Without LiDAR (Temporary)"
echo "    You can still use the robot for testing:"
echo "      - Motors and base control will work"
echo "      - Camera will work"
echo "      - IMU will work"
echo "    Just disable LiDAR in launch file"
echo ""

echo -e "${BLUE}======================================================================${NC}"
