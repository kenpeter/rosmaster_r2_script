#!/bin/bash
# Sensor Test Script (Non-Moving)
# Tests Camera, LiDAR, and IMU sensors WITHOUT moving the robot

echo "========================================="
echo "  Sensor Test Suite (Non-Moving)"
echo "========================================="
echo ""
echo "This script will test sensors WITHOUT moving the robot:"
echo "  ✅ Camera feed (/RGBD/RGB/Image)"
echo "  ✅ LiDAR sensor (/scan)"
echo "  ✅ IMU sensor (/imu/data_raw)"
echo ""

# Source ROS2 environment
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"
source "$WORKSPACE_ROOT/install/setup.bash"

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Test results
CAMERA_PASS=0
LIDAR_PASS=0
IMU_PASS=0

echo ""
echo "========================================="
echo " System Information"
echo "========================================="
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "Robot Type: $ROBOT_TYPE"
echo "LiDAR Type: $RPLIDAR_TYPE"
echo "Camera Type: $CAMERA_TYPE"
echo "Workspace: $WORKSPACE_ROOT"
echo ""

# Check if robot is running
echo "Checking if robot hardware is running..."
if ! ros2 node list &>/dev/null; then
    echo -e "${RED}❌ No ROS2 nodes detected!${NC}"
    echo ""
    echo "Please start robot hardware first:"
    echo "  Terminal 1:"
    echo "  ros2 launch yahboomcar_bringup yahboomcar_bringup_R2_launch.py  # or X3"
    echo "  or"
    echo "  ros2 launch yahboomcar_bringup yahboomcar_bringup_R2_launch.py"
    exit 1
fi

echo -e "${GREEN}✅ ROS2 system is running${NC}"
echo ""
echo "Active nodes:"
ros2 node list
echo ""

read -p "Press ENTER to start sensor tests..."

# ==========================================
# Test 1: Camera
# ==========================================
echo ""
echo "========================================="
echo " Test 1: Camera Feed"
echo "========================================="
echo ""

# Check for camera topic (try multiple possible names)
CAMERA_TOPIC="/RGBD/RGB/Image"
if ! ros2 topic list | grep -q "$CAMERA_TOPIC"; then
    # Try alternative Astra camera topic
    if ros2 topic list | grep -q "/camera/color/image_raw"; then
        CAMERA_TOPIC="/camera/color/image_raw"
    fi
fi

if ros2 topic list | grep -q "$CAMERA_TOPIC"; then
    echo -e "${GREEN}✅ Camera topic found: $CAMERA_TOPIC${NC}"

    echo "Checking camera info..."
    ros2 topic info $CAMERA_TOPIC --verbose

    echo ""
    echo "Checking camera rate..."
    CAMERA_RATE=$(timeout 3 ros2 topic hz $CAMERA_TOPIC 2>&1 | grep "average rate" | awk '{print $3}')
    if [ -n "$CAMERA_RATE" ]; then
        echo -e "${GREEN}✅ Camera publishing at ~$CAMERA_RATE Hz${NC}"
        CAMERA_PASS=1
    else
        echo -e "${YELLOW}⚠️  Camera topic exists but may not be publishing${NC}"
    fi

    echo ""
    echo "Reading one camera frame..."
    if timeout 3 ros2 topic echo $CAMERA_TOPIC --once &>/dev/null; then
        echo -e "${GREEN}✅ Camera data readable${NC}"
    else
        echo -e "${RED}❌ Could not read camera data${NC}"
        CAMERA_PASS=0
    fi
else
    echo -e "${RED}❌ Camera topic not found!${NC}"
    echo "Available image topics:"
    ros2 topic list | grep -i image
fi

# ==========================================
# Test 2: LiDAR
# ==========================================
echo ""
echo "========================================="
echo " Test 2: LiDAR Sensor"
echo "========================================="
echo ""

LIDAR_TOPIC="/scan"
if ros2 topic list | grep -q "^$LIDAR_TOPIC$"; then
    echo -e "${GREEN}✅ LiDAR topic found: $LIDAR_TOPIC${NC}"

    echo "Checking LiDAR info..."
    ros2 topic info $LIDAR_TOPIC --verbose

    echo ""
    echo "Checking LiDAR rate..."
    LIDAR_RATE=$(timeout 3 ros2 topic hz $LIDAR_TOPIC 2>&1 | grep "average rate" | awk '{print $3}')
    if [ -n "$LIDAR_RATE" ]; then
        echo -e "${GREEN}✅ LiDAR publishing at ~$LIDAR_RATE Hz${NC}"
    else
        echo -e "${YELLOW}⚠️  LiDAR topic exists but may not be publishing${NC}"
    fi

    echo ""
    echo "Reading one LiDAR scan..."
    if timeout 3 ros2 topic echo $LIDAR_TOPIC --once &>/dev/null; then
        echo -e "${GREEN}✅ LiDAR data readable${NC}"
        LIDAR_PASS=1
    else
        echo -e "${RED}❌ Could not read LiDAR data${NC}"
    fi
else
    echo -e "${RED}❌ LiDAR topic not found!${NC}"
fi

# ==========================================
# Test 3: IMU
# ==========================================
echo ""
echo "========================================="
echo " Test 3: IMU Sensor"
echo "========================================="
echo ""

IMU_TOPIC="/imu/data_raw"
if ros2 topic list | grep -q "^$IMU_TOPIC$"; then
    echo -e "${GREEN}✅ IMU topic found: $IMU_TOPIC${NC}"

    echo "Checking IMU info..."
    ros2 topic info $IMU_TOPIC --verbose

    echo ""
    echo "Checking IMU rate..."
    IMU_RATE=$(timeout 3 ros2 topic hz $IMU_TOPIC 2>&1 | grep "average rate" | awk '{print $3}')
    if [ -n "$IMU_RATE" ]; then
        echo -e "${GREEN}✅ IMU publishing at ~$IMU_RATE Hz${NC}"
    else
        echo -e "${YELLOW}⚠️  IMU topic exists but may not be publishing${NC}"
    fi

    echo ""
    echo "Reading one IMU sample..."
    if timeout 3 ros2 topic echo $IMU_TOPIC --once &>/dev/null; then
        echo -e "${GREEN}✅ IMU data readable${NC}"
        IMU_PASS=1
    else
        echo -e "${RED}❌ Could not read IMU data${NC}"
    fi

    echo ""
    echo -e "${BLUE}IMU Data Sample:${NC}"
    timeout 3 ros2 topic echo $IMU_TOPIC --once 2>/dev/null | head -30
else
    echo -e "${RED}❌ IMU topic not found!${NC}"
fi

# ==========================================
# Test Summary
# ==========================================
echo ""
echo "========================================="
echo " Sensor Test Summary"
echo "========================================="
echo ""

TOTAL_TESTS=3
PASSED_TESTS=0

echo -n "Camera:  "
if [ $CAMERA_PASS -eq 1 ]; then
    echo -e "${GREEN}✅ PASS${NC}"
    ((PASSED_TESTS++))
else
    echo -e "${RED}❌ FAIL${NC}"
fi

echo -n "LiDAR:   "
if [ $LIDAR_PASS -eq 1 ]; then
    echo -e "${GREEN}✅ PASS${NC}"
    ((PASSED_TESTS++))
else
    echo -e "${RED}❌ FAIL${NC}"
fi

echo -n "IMU:     "
if [ $IMU_PASS -eq 1 ]; then
    echo -e "${GREEN}✅ PASS${NC}"
    ((PASSED_TESTS++))
else
    echo -e "${RED}❌ FAIL${NC}"
fi

echo ""
echo "Result: $PASSED_TESTS/$TOTAL_TESTS sensor tests passed"
echo ""

# ==========================================
# Visualization Options
# ==========================================
echo "========================================="
echo " Visualization & Monitoring Options"
echo "========================================="
echo ""
echo "View camera feed:"
echo "  ros2 run rqt_image_view rqt_image_view $CAMERA_TOPIC"
echo ""
echo "View LiDAR in RViz:"
echo "  rviz2"
echo "  Then: Add -> LaserScan -> Topic: $LIDAR_TOPIC"
echo ""
echo "Monitor IMU data:"
echo "  ros2 topic echo $IMU_TOPIC"
echo "  ros2 topic echo $IMU_TOPIC --field linear_acceleration"
echo "  ros2 topic echo $IMU_TOPIC --field angular_velocity"
echo ""
echo "Check topic rates:"
echo "  ros2 topic hz $CAMERA_TOPIC"
echo "  ros2 topic hz $LIDAR_TOPIC"
echo "  ros2 topic hz $IMU_TOPIC"
echo ""

if [ $PASSED_TESTS -eq $TOTAL_TESTS ]; then
    echo -e "${GREEN}=========================================${NC}"
    echo -e "${GREEN} 🎉 All sensor tests PASSED!${NC}"
    echo -e "${GREEN}=========================================${NC}"
    echo ""
    echo "✅ Sensors are working correctly"
    echo "➡️  Next step: Test motors with ./test_motor.sh"
    exit 0
else
    echo -e "${YELLOW}=========================================${NC}"
    echo -e "${YELLOW} ⚠️  Some sensor tests failed${NC}"
    echo -e "${YELLOW}=========================================${NC}"
    echo ""
    echo "Troubleshooting tips:"
    echo "  - Check cable connections"
    echo "  - Verify correct launch file for your robot model"
    echo "  - Check for error messages in robot launch terminal"
    echo "  - Try restarting robot hardware"
    exit 1
fi
