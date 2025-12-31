#!/bin/bash
# Quick sensor verification script

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
NC='\033[0m'

echo -e "${YELLOW}=======================================${NC}"
echo -e "${YELLOW}  Sensor Test Script${NC}"
echo -e "${YELLOW}=======================================${NC}"
echo ""

# Setup environment
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=28

echo -e "${YELLOW}Checking for sensor topics...${NC}"
echo ""

# Check LIDAR
if timeout 3 ros2 topic hz /scan 2>&1 | grep -q "average rate"; then
    RATE=$(timeout 3 ros2 topic hz /scan 2>&1 | grep "average rate" | awk '{print $3}')
    echo -e "${GREEN}✓ LIDAR: /scan publishing at ${RATE} Hz${NC}"
else
    echo -e "${RED}✗ LIDAR: /scan NOT publishing${NC}"
fi

# Check Camera Color
if timeout 3 ros2 topic hz /camera/color/image_raw 2>&1 | grep -q "average rate"; then
    RATE=$(timeout 3 ros2 topic hz /camera/color/image_raw 2>&1 | grep "average rate" | awk '{print $3}')
    echo -e "${GREEN}✓ Camera (color): /camera/color/image_raw publishing at ${RATE} Hz${NC}"
else
    echo -e "${RED}✗ Camera (color): /camera/color/image_raw NOT publishing${NC}"
fi

# Check Camera Depth
if timeout 3 ros2 topic hz /camera/depth/image_raw 2>&1 | grep -q "average rate"; then
    RATE=$(timeout 3 ros2 topic hz /camera/depth/image_raw 2>&1 | grep "average rate" | awk '{print $3}')
    echo -e "${GREEN}✓ Camera (depth): /camera/depth/image_raw publishing at ${RATE} Hz${NC}"
else
    echo -e "${YELLOW}! Camera (depth): /camera/depth/image_raw NOT publishing (IR sensor may be broken)${NC}"
fi

# Check Odometry
if timeout 3 ros2 topic hz /odom 2>&1 | grep -q "average rate"; then
    RATE=$(timeout 3 ros2 topic hz /odom 2>&1 | grep "average rate" | awk '{print $3}')
    echo -e "${GREEN}✓ Odometry: /odom publishing at ${RATE} Hz${NC}"
else
    echo -e "${RED}✗ Odometry: /odom NOT publishing${NC}"
fi

echo ""
echo -e "${YELLOW}=======================================${NC}"
