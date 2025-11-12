#!/bin/bash
# Test RPLiDAR Connection and Motor Status

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}=========================================${NC}"
echo -e "${BLUE}  RPLiDAR A1 Diagnostic Test${NC}"
echo -e "${BLUE}=========================================${NC}"
echo ""

# Check device exists
if [ -e "/dev/rplidar" ]; then
    echo -e "${GREEN}✅ /dev/rplidar exists${NC}"
    ls -la /dev/rplidar
else
    echo -e "${RED}❌ /dev/rplidar not found${NC}"
    exit 1
fi

echo ""

# Check device info
echo -e "${YELLOW}Device Information:${NC}"
udevadm info /dev/rplidar | grep -E "ID_VENDOR|ID_MODEL|ID_SERIAL"

echo ""

# Check permissions
echo -e "${YELLOW}Permissions:${NC}"
if [ -r "/dev/rplidar" ] && [ -w "/dev/rplidar" ]; then
    echo -e "${GREEN}✅ Read/Write access OK${NC}"
else
    echo -e "${RED}❌ Insufficient permissions${NC}"
    echo "Run: sudo chmod 666 /dev/rplidar"
fi

echo ""
echo -e "${BLUE}=========================================${NC}"
echo -e "${BLUE}  Testing RPLiDAR Communication${NC}"
echo -e "${BLUE}=========================================${NC}"
echo ""
echo "This will attempt to start the RPLiDAR for 5 seconds..."
echo "You should hear the motor spin and see scan data."
echo ""
echo "Press Ctrl+C to cancel, or Enter to continue..."
read

# Source ROS2
source /opt/ros/humble/setup.bash
cd /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws
source install/setup.bash

# Test launch
timeout 5 ros2 run sllidar_ros2 sllidar_node --ros-args \
    -p serial_port:=/dev/rplidar \
    -p serial_baudrate:=115200 \
    -p frame_id:=laser

RESULT=$?

echo ""
echo -e "${BLUE}=========================================${NC}"
echo -e "${BLUE}  Test Result${NC}"
echo -e "${BLUE}=========================================${NC}"

if [ $RESULT -eq 124 ]; then
    echo -e "${GREEN}✅ Test completed (timeout - expected)${NC}"
    echo "Check above for any errors or scan data."
elif [ $RESULT -eq 0 ]; then
    echo -e "${GREEN}✅ Node started successfully${NC}"
else
    echo -e "${RED}❌ Test failed with exit code: $RESULT${NC}"
    echo ""
    echo "Common issues:"
    echo "1. Motor not spinning - check power connection"
    echo "2. Wrong device - verify /dev/rplidar is correct"
    echo "3. Baudrate mismatch - RPLiDAR A1 uses 115200"
fi

echo ""
