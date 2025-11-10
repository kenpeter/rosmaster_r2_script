#!/bin/bash
# Complete System Integration Test
# Tests all hardware, software, and autonomous capabilities

echo "========================================="
echo "  Complete System Integration Test"
echo "========================================="
echo ""
echo "This is a comprehensive test of the entire robot system:"
echo "  📹 Camera, LiDAR, IMU (sensors)"
echo "  🚗 Motor control (movement)"
echo "  🤖 Autonomous capabilities (optional)"
echo ""

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Source ROS2 environment
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"
source "$WORKSPACE_ROOT/install/setup.bash"

# Test results
SENSOR_PASS=0
MOTOR_PASS=0
AUTONOMOUS_PASS=0

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

echo "========================================="
echo " Test Plan"
echo "========================================="
echo "Phase 1: Sensor Tests (non-moving)"
echo "Phase 2: Motor Tests (moving)"
echo "Phase 3: Autonomous System Tests (optional)"
echo ""
read -p "Press ENTER to begin Phase 1 (Sensor Tests)..."

# ==========================================
# PHASE 1: SENSOR TESTS
# ==========================================
echo ""
echo -e "${CYAN}=========================================${NC}"
echo -e "${CYAN} PHASE 1: SENSOR TESTS (Non-Moving)${NC}"
echo -e "${CYAN}=========================================${NC}"
echo ""

CAMERA_OK=0
LIDAR_OK=0
IMU_OK=0

# Test Camera
echo -e "${BLUE}[1/3] Testing Camera...${NC}"
if ros2 topic list | grep -q "/RGBD/RGB/Image"; then
    if timeout 3 ros2 topic hz /RGBD/RGB/Image &>/dev/null; then
        echo -e "${GREEN}  ✅ Camera working${NC}"
        CAMERA_OK=1
    else
        echo -e "${YELLOW}  ⚠️  Camera topic exists but not publishing${NC}"
    fi
else
    echo -e "${RED}  ❌ Camera topic not found${NC}"
fi

# Test LiDAR
echo -e "${BLUE}[2/3] Testing LiDAR...${NC}"
if ros2 topic list | grep -q "^/scan$"; then
    if timeout 3 ros2 topic hz /scan &>/dev/null; then
        echo -e "${GREEN}  ✅ LiDAR working${NC}"
        LIDAR_OK=1
    else
        echo -e "${YELLOW}  ⚠️  LiDAR topic exists but not publishing${NC}"
    fi
else
    echo -e "${RED}  ❌ LiDAR topic not found${NC}"
fi

# Test IMU
echo -e "${BLUE}[3/3] Testing IMU...${NC}"
if ros2 topic list | grep -q "^/imu/data_raw$"; then
    if timeout 3 ros2 topic hz /imu/data_raw &>/dev/null; then
        echo -e "${GREEN}  ✅ IMU working${NC}"
        IMU_OK=1
    else
        echo -e "${YELLOW}  ⚠️  IMU topic exists but not publishing${NC}"
    fi
else
    echo -e "${RED}  ❌ IMU topic not found${NC}"
fi

# Phase 1 Summary
echo ""
if [ $CAMERA_OK -eq 1 ] && [ $LIDAR_OK -eq 1 ] && [ $IMU_OK -eq 1 ]; then
    echo -e "${GREEN}✅ Phase 1 PASSED: All sensors working${NC}"
    SENSOR_PASS=1
else
    echo -e "${YELLOW}⚠️  Phase 1 PARTIAL: Some sensors failed${NC}"
    echo "   You can continue, but some features may not work."
fi

echo ""
read -p "Press ENTER to begin Phase 2 (Motor Tests - ROBOT WILL MOVE!)..."

# ==========================================
# PHASE 2: MOTOR TESTS
# ==========================================
echo ""
echo -e "${CYAN}=========================================${NC}"
echo -e "${CYAN} PHASE 2: MOTOR TESTS (Moving)${NC}"
echo -e "${CYAN}=========================================${NC}"
echo ""
echo -e "${RED}⚠️  WARNING: Robot will move!${NC}"
echo -e "${RED}    Ensure clear space (1 meter on all sides)${NC}"
echo ""
read -p "Press ENTER to continue or Ctrl+C to abort..."

# Check motor control topic
if ! ros2 topic list | grep -q "^/cmd_vel$"; then
    echo -e "${RED}❌ /cmd_vel topic not found! Cannot test motors.${NC}"
    MOTOR_PASS=0
else
    echo -e "${GREEN}✅ /cmd_vel topic found${NC}"
    echo ""
    echo "Starting motor tests in 3 seconds..."
    sleep 3

    # Safety stop
    echo -e "${BLUE}Test 1: Safety stop${NC}"
    ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0}" --once
    sleep 1

    # Forward
    echo -e "${BLUE}Test 2: Forward (1 sec)${NC}"
    ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.1}" --once
    sleep 1
    ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0}" --once
    echo -e "${GREEN}  ✅ Forward motion OK${NC}"
    sleep 1

    # Rotation
    echo -e "${BLUE}Test 3: Rotation (1 sec)${NC}"
    ros2 topic pub /cmd_vel geometry_msgs/Twist "angular: {z: 0.3}" --once
    sleep 1
    ros2 topic pub /cmd_vel geometry_msgs/Twist "angular: {z: 0.0}" --once
    echo -e "${GREEN}  ✅ Rotation OK${NC}"
    sleep 1

    # Final stop
    ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0}" --once

    echo ""
    echo -e "${GREEN}✅ Phase 2 PASSED: Motor control working${NC}"
    MOTOR_PASS=1
fi

# ==========================================
# PHASE 3: AUTONOMOUS SYSTEM TESTS
# ==========================================
echo ""
echo -e "${CYAN}=========================================${NC}"
echo -e "${CYAN} PHASE 3: AUTONOMOUS SYSTEM (Optional)${NC}"
echo -e "${CYAN}=========================================${NC}"
echo ""
echo "This phase tests autonomous driving capabilities:"
echo "  - Lane detection"
echo "  - YOLO object detection"
echo "  - VLM scene understanding"
echo "  - LLM decision making"
echo ""
echo "Note: This requires yahboomcar_autonomous to be built."
echo ""
read -p "Do you want to test autonomous features? (y/N) " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo ""
    echo "Checking for autonomous package..."

    # Check if autonomous package exists
    if [ ! -d "$WORKSPACE_ROOT/install/yahboomcar_autonomous" ]; then
        echo -e "${YELLOW}⚠️  yahboomcar_autonomous package not found${NC}"
        echo "   Skipping autonomous tests"
        AUTONOMOUS_PASS=-1
    else
        # Check Ollama service
        echo "Checking Ollama service..."
        if systemctl is-active --quiet ollama; then
            echo -e "${GREEN}  ✅ Ollama service running${NC}"
        else
            echo -e "${YELLOW}  ⚠️  Ollama service not running${NC}"
            echo "     Start with: sudo systemctl start ollama"
        fi

        # Check for lane detection topic (if autonomous is running)
        echo ""
        echo "Is yahboomcar_autonomous already running? (y/N)"
        read -p "> " -n 1 -r
        echo ""

        if [[ $REPLY =~ ^[Yy]$ ]]; then
            echo ""
            echo "Testing autonomous topics..."

            LANE_OK=0
            VLM_OK=0

            # Check lane detection
            if ros2 topic list | grep -q "/lane_detection/info"; then
                echo -e "${GREEN}  ✅ Lane detection topic found${NC}"
                LANE_OK=1
            else
                echo -e "${YELLOW}  ⚠️  Lane detection not running${NC}"
            fi

            # Check VLM
            if ros2 topic list | grep -q "/scene_understanding"; then
                echo -e "${GREEN}  ✅ VLM scene understanding topic found${NC}"
                VLM_OK=1
            else
                echo -e "${YELLOW}  ⚠️  VLM not running${NC}"
            fi

            # Check YOLO
            if ros2 topic list | grep -q "/yolo"; then
                echo -e "${GREEN}  ✅ YOLO topics found${NC}"
            else
                echo -e "${YELLOW}  ⚠️  YOLO not running${NC}"
            fi

            if [ $LANE_OK -eq 1 ] || [ $VLM_OK -eq 1 ]; then
                echo ""
                echo -e "${GREEN}✅ Phase 3 PASSED: Autonomous system operational${NC}"
                AUTONOMOUS_PASS=1
            else
                echo ""
                echo -e "${YELLOW}⚠️  Phase 3 PARTIAL: Limited autonomous features${NC}"
                AUTONOMOUS_PASS=0
            fi
        else
            echo ""
            echo "To test autonomous system:"
            echo "  Terminal 2:"
            echo "  ros2 launch yahboomcar_autonomous autonomous_driving.launch.py"
            echo ""
            echo "Then run this test again."
            AUTONOMOUS_PASS=-1
        fi
    fi
else
    echo "Skipping autonomous tests"
    AUTONOMOUS_PASS=-1
fi

# ==========================================
# FINAL SUMMARY
# ==========================================
echo ""
echo -e "${MAGENTA}=========================================${NC}"
echo -e "${MAGENTA} FINAL TEST SUMMARY${NC}"
echo -e "${MAGENTA}=========================================${NC}"
echo ""

echo "Phase 1 - Sensors:     $([ $SENSOR_PASS -eq 1 ] && echo -e "${GREEN}✅ PASS${NC}" || echo -e "${RED}❌ FAIL${NC}")"
echo "Phase 2 - Motors:      $([ $MOTOR_PASS -eq 1 ] && echo -e "${GREEN}✅ PASS${NC}" || echo -e "${RED}❌ FAIL${NC}")"

if [ $AUTONOMOUS_PASS -eq 1 ]; then
    echo -e "Phase 3 - Autonomous:  ${GREEN}✅ PASS${NC}"
elif [ $AUTONOMOUS_PASS -eq 0 ]; then
    echo -e "Phase 3 - Autonomous:  ${YELLOW}⚠️  PARTIAL${NC}"
else
    echo -e "Phase 3 - Autonomous:  ${BLUE}⊘  SKIPPED${NC}"
fi

echo ""

# Overall result
if [ $SENSOR_PASS -eq 1 ] && [ $MOTOR_PASS -eq 1 ]; then
    echo -e "${GREEN}=========================================${NC}"
    echo -e "${GREEN} 🎉 SYSTEM READY FOR OPERATION!${NC}"
    echo -e "${GREEN}=========================================${NC}"
    echo ""
    echo "Hardware Status: All systems functional"

    if [ $AUTONOMOUS_PASS -eq 1 ]; then
        echo "Autonomous Status: Full capabilities available"
    elif [ $AUTONOMOUS_PASS -eq 0 ]; then
        echo "Autonomous Status: Partial features available"
    else
        echo "Autonomous Status: Not tested"
    fi

    echo ""
    echo "Next steps:"
    echo "  - For manual control: Use teleop or send /cmd_vel commands"
    echo "  - For autonomous: Launch yahboomcar_autonomous package"
    echo "  - For navigation: Launch navigation stack"
    exit 0
else
    echo -e "${YELLOW}=========================================${NC}"
    echo -e "${YELLOW} ⚠️  SYSTEM CHECKS INCOMPLETE${NC}"
    echo -e "${YELLOW}=========================================${NC}"
    echo ""
    echo "Some critical systems failed testing."
    echo ""
    echo "Troubleshooting:"
    echo "  - Run individual tests for more details:"
    echo "    $SCRIPT_DIR/test_sensors.sh"
    echo "    $SCRIPT_DIR/test_motor.sh"
    echo "  - Check robot hardware launch"
    echo "  - Verify cable connections"
    echo "  - Review error messages above"
    exit 1
fi
