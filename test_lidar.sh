#!/bin/bash
# LiDAR Debug and Test Script
# Comprehensive debugging for RPLiDAR issues

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${BLUE}=========================================${NC}"
echo -e "${BLUE}  LiDAR Debug and Test Script${NC}"
echo -e "${BLUE}=========================================${NC}"
echo ""

# Source ROS2 environment
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"
source "$WORKSPACE_ROOT/install/setup.bash"

# ========================================
# Hardware Checks First
# ========================================
echo -e "${CYAN}Step 0: Hardware Checks${NC}"
echo "-------------------------------------------"

# Check if device exists
if [ -e /dev/rplidar ]; then
    echo -e "${GREEN}✅ /dev/rplidar exists${NC}"
    ls -la /dev/rplidar
    ACTUAL_DEV=$(readlink -f /dev/rplidar)
    echo "   Actual device: $ACTUAL_DEV"
else
    echo -e "${RED}❌ /dev/rplidar NOT FOUND${NC}"
    echo ""
    echo "Available USB serial devices:"
    ls -la /dev/ttyUSB* 2>/dev/null || echo "No /dev/ttyUSB* devices"
    exit 1
fi

# Check USB connection
echo ""
echo "USB Device Info:"
lsusb | grep -i "10c4:ea60"
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ RPLiDAR USB device detected${NC}"
else
    echo -e "${YELLOW}⚠️  RPLiDAR USB device (10c4:ea60) not found${NC}"
fi

# Check device permissions
echo ""
echo "Device permissions:"
ls -la $ACTUAL_DEV

# Check if device is in use
echo ""
if lsof $ACTUAL_DEV 2>/dev/null; then
    echo -e "${YELLOW}⚠️  Device is currently in use!${NC}"
    lsof $ACTUAL_DEV
    echo ""
    echo "You may need to stop the robot first:"
    echo "  ./cleanup_robot.sh"
    exit 1
else
    echo -e "${GREEN}✅ Device is not in use (ready for testing)${NC}"
fi

# Ask about motor
echo ""
echo -e "${YELLOW}CRITICAL CHECK: Is the RPLiDAR motor spinning?${NC}"
echo "  - Listen: Do you hear a high-pitched whirring sound?"
echo "  - Look: Is the top of the LiDAR rotating?"
echo ""
read -p "Is the motor spinning? (y/N): " -n 1 -r
echo ""

if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo ""
    echo -e "${RED}⚠️  Motor is NOT spinning - this will cause timeout errors!${NC}"
    echo ""
    echo "The motor must spin for LiDAR to work. Common causes:"
    echo "  1. Insufficient USB power"
    echo "  2. Bad USB cable"
    echo "  3. LiDAR hardware issue"
    echo ""
    echo "Try:"
    echo "  - Different USB port (try USB 2.0)"
    echo "  - Powered USB hub"
    echo "  - Different USB cable"
    echo ""
    read -p "Continue testing anyway? (y/N): " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo ""
echo -e "${CYAN}Step 1: Direct Serial Communication Test${NC}"
echo "-------------------------------------------"

# Test if we can read from the device
echo "Testing serial port read..."
if timeout 2 cat $ACTUAL_DEV > /dev/null 2>&1; then
    echo -e "${GREEN}✅ Can open serial port${NC}"
else
    echo -e "${YELLOW}⚠️  Cannot read from serial port (may be normal if not sending data)${NC}"
fi

# Check serial port settings
echo ""
echo "Current serial port settings:"
stty -F $ACTUAL_DEV 2>/dev/null || echo "Could not read settings"

echo ""
echo -e "${CYAN}Step 2: Test RPLiDAR Node Standalone${NC}"
echo "-------------------------------------------"
echo ""
echo "Launching RPLiDAR node with verbose logging..."
echo "This will run for 10 seconds..."
echo ""

# Run the node directly with logging
timeout 10 ros2 run sllidar_ros2 sllidar_node \
    --ros-args \
    -p serial_port:=/dev/rplidar \
    -p serial_baudrate:=115200 \
    -p frame_id:=laser \
    -p inverted:=false \
    -p angle_compensate:=true \
    --log-level debug \
    2>&1 || echo ""

echo ""
echo -e "${CYAN}Step 3: Checking if node is still running${NC}"
echo "-------------------------------------------"

# Check if topic was created
if ros2 topic list | grep -q "^/scan$"; then
    echo -e "${GREEN}✅ /scan topic found!${NC}"

    echo ""
    echo "Testing topic data..."
    timeout 3 ros2 topic echo /scan --once > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ LiDAR is publishing data!${NC}"
    else
        echo -e "${RED}❌ /scan topic exists but no data${NC}"
    fi
else
    echo -e "${RED}❌ /scan topic NOT found${NC}"
fi

echo ""
echo -e "${CYAN}=========================================${NC}"
echo -e "${CYAN}  Debug Summary and Recommendations${NC}"
echo -e "${CYAN}=========================================${NC}"
echo ""

# Summary
echo -e "${BLUE}If the test above failed with timeout:${NC}"
echo ""
echo "1. Motor not spinning:"
echo "   - Most common cause of timeout errors"
echo "   - Try different USB port (USB 2.0 often better)"
echo "   - Use powered USB hub"
echo "   - Check if LiDAR needs external 5V power"
echo ""
echo "2. Wrong baud rate:"
echo "   - A1 uses 115200 (current setting)"
echo "   - A2 uses 256000"
echo "   - Try: ros2 run sllidar_ros2 sllidar_node -p serial_baudrate:=256000"
echo ""
echo "3. Communication issues:"
echo "   - Bad USB cable"
echo "   - USB power fluctuations"
echo "   - Try shorter cable"
echo ""
echo "4. Device permissions:"
echo "   - Check: ls -la $ACTUAL_DEV"
echo "   - Should be readable/writable by user"
echo "   - User should be in dialout group"
echo ""

echo -e "${BLUE}To test with different baud rate:${NC}"
echo "  ros2 run sllidar_ros2 sllidar_node \\"
echo "    --ros-args -p serial_port:=/dev/rplidar -p serial_baudrate:=256000"
echo ""

echo -e "${BLUE}To monitor kernel messages for USB issues:${NC}"
echo "  sudo dmesg | tail -20"
echo ""

echo -e "${BLUE}To verify hardware manually:${NC}"
echo "  # Install minicom: sudo apt install minicom"
echo "  # Test serial: sudo minicom -D /dev/rplidar -b 115200"
echo ""

echo "========================================="
