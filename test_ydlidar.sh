#!/bin/bash
###############################################################################
# YDLIDAR X4 Test Script
# Tests the YDLIDAR X4 with ROS2 driver
###############################################################################

set -e  # Exit on error

echo "======================================================================"
echo "  YDLIDAR X4 Test Script"
echo "======================================================================"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if YDLIDAR device exists
echo "[1/3] Checking YDLIDAR device..."
if [ -e /dev/ydlidar ]; then
    echo -e "${GREEN}✅ /dev/ydlidar found${NC}"
    ls -la /dev/ydlidar
    echo ""
else
    echo -e "${RED}❌ /dev/ydlidar not found${NC}"
    echo "Please check USB connection and udev rules"
    exit 1
fi

# Check USB connection
echo "[2/3] USB Device Info..."
lsusb | grep -i "QinHeng\|1a86" || echo "YDLIDAR not found in lsusb"
echo ""

# Launch YDLIDAR node
echo "[3/3] Launching YDLIDAR ROS2 node..."
echo -e "${YELLOW}This will run for 15 seconds to collect scan data${NC}"
echo "You should hear the motor spinning..."
echo ""

cd /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws
source install/setup.bash

# Launch and monitor
timeout 15 ros2 launch yahboomcar_laser ydlidar_x4_launch.py &
LAUNCH_PID=$!

# Wait a bit for startup
sleep 3

# Check if scan topic is publishing
echo ""
echo "Checking /scan topic..."
timeout 5 ros2 topic hz /scan 2>&1 || echo "Waiting for scan data..."

echo ""
echo "Checking scan message:"
timeout 3 ros2 topic echo /scan --once 2>&1 | head -20 || echo "No scan data yet"

# Wait for timeout
wait $LAUNCH_PID 2>/dev/null || true

echo ""
echo "======================================================================"
echo "Test completed!"
echo ""
echo "If you saw scan data above, the YDLIDAR X4 is working correctly!"
echo "======================================================================"
