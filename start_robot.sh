#!/bin/bash
# Start Yahboom R2 Robot - Full System with Auto-Cleanup

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

echo -e "${BLUE}=========================================${NC}"
echo -e "${BLUE}  Yahboom Rosmaster R2 - Full Launch${NC}"
echo -e "${BLUE}=========================================${NC}"
echo ""
echo "This will start:"
echo "  ✅ R2 Robot Hardware (Ackermann steering)"
echo "  ✅ RPLiDAR A1"
echo "  ✅ Astra Plus Camera"
echo "  ✅ IMU"
echo ""

# ========================================
# Step 1: Stop interfering processes
# ========================================
echo -e "${YELLOW}Step 1: Stopping interfering processes...${NC}"

# Stop OLED display process (can interfere with serial port)
if pgrep -f yahboom_oled > /dev/null; then
    echo "  Stopping yahboom_oled process..."
    pkill -f yahboom_oled
    sleep 0.5
    echo -e "${GREEN}  ✅ OLED process stopped${NC}"
fi

# Cleanup any existing robot processes
# "$SCRIPT_DIR/cleanup_robot.sh"

# Wait a moment for cleanup to complete
sleep 1

# ========================================
# Step 2: Check hardware connectivity
# ========================================
echo -e "${YELLOW}Step 2: Checking device permissions...${NC}"

# Check robot controller
if [ -e "/dev/myserial" ]; then
    echo -e "${GREEN}✅ Robot controller connected${NC}"
else
    echo -e "${RED}❌ Robot controller not found (/dev/myserial)${NC}"
    echo "   Check USB cable and power"
fi

# Check LiDAR
if [ -e "/dev/rplidar" ]; then
    echo -e "${GREEN}✅ RPLiDAR connected${NC}"
    echo -e "${YELLOW}   NOTE: Ensure RPLiDAR motor is spinning!${NC}"
elif [ -e "/dev/ttyUSB0" ] || [ -e "/dev/ttyUSB1" ]; then
    sudo chmod 666 /dev/ttyUSB* 2>/dev/null
    echo -e "${YELLOW}⚠️  Found USB devices but no /dev/rplidar symlink${NC}"
else
    echo -e "${YELLOW}⚠️  No /dev/ttyUSB* found - LiDAR may not be connected${NC}"
fi

# Check camera
if lsusb | grep -q -E "Orbbec|Astra|2bc5"; then
    echo -e "${GREEN}✅ Camera detected${NC}"
else
    echo -e "${YELLOW}⚠️  Camera not detected${NC}"
fi

echo ""

# ========================================
# Step 3: Setup environment
# ========================================
echo -e "${YELLOW}Step 3: Loading ROS2 environment...${NC}"

source /opt/ros/humble/setup.bash
# Source library workspace first (contains robot_localization)
source /home/jetson/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
cd "$WORKSPACE_ROOT"
source install/setup.bash
export ROS_DOMAIN_ID=28

echo -e "${GREEN}✅ Environment loaded${NC}"
echo ""

# ========================================
# Step 4: Launch robot system
# ========================================
echo -e "${GREEN}=========================================${NC}"
echo -e "${GREEN}🚀 Launching full system...${NC}"
echo -e "${GREEN}=========================================${NC}"
echo ""
echo "Press Ctrl+C to stop the robot"
echo ""

# Launch R2 robot with all sensors
ros2 launch yahboomcar_bringup yahboomcar_bringup_R2_full_launch.py
