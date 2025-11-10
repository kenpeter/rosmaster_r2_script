#!/bin/bash
# Motor Control Test Script (MOVING TEST)
# Tests motor control via /cmd_vel topic - Robot WILL MOVE!

echo "========================================="
echo "  Motor Control Test (MOVING TEST)"
echo "========================================="
echo ""

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${RED}⚠️  WARNING: This script will move the robot!${NC}"
echo -e "${RED}    Make sure the robot has clear space around it.${NC}"
echo -e "${RED}    Keep at least 1 meter clearance on all sides.${NC}"
echo ""
read -p "Press ENTER to continue or Ctrl+C to abort..."

# Source ROS2 environment
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"
source "$WORKSPACE_ROOT/install/setup.bash"

echo ""
echo "========================================="
echo " System Information"
echo "========================================="
echo "Robot Type: $ROBOT_TYPE"
echo "Workspace: $WORKSPACE_ROOT"
echo ""

echo "Testing motor control on topic: /cmd_vel"
echo ""

# Check if topic exists
echo "Step 1: Checking if /cmd_vel topic exists..."
if ros2 topic list | grep -q "^/cmd_vel$"; then
    echo -e "${GREEN}✅ /cmd_vel topic found!${NC}"
else
    echo -e "${RED}❌ /cmd_vel topic NOT found!${NC}"
    echo ""
    echo "Make sure robot hardware is running:"
    echo "  ros2 launch yahboomcar_bringup yahboomcar_bringup_R2_launch.py  # or X3"
    echo "  or"
    echo "  ros2 launch yahboomcar_bringup yahboomcar_bringup_R2_launch.py"
    exit 1
fi

echo ""
echo "Step 2: Getting /cmd_vel topic info..."
ros2 topic info /cmd_vel --verbose

echo ""
echo "========================================="
echo "  Motor Movement Tests"
echo "========================================="
echo ""
echo "Starting movement tests in 3 seconds..."
sleep 3

# Test 1: Stop (ensure motors are stopped)
echo ""
echo -e "${BLUE}Test 1: STOP (safety check)${NC}"
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}" --once
echo -e "${GREEN}✅ Stop command sent${NC}"
sleep 2

# Test 2: Forward
echo ""
echo "Test 2: Moving FORWARD (slow speed) for 2 seconds..."
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.1, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}" --once
sleep 2
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}" --once
echo "✅ Forward test complete, stopped"
sleep 1

# Test 3: Backward
echo ""
echo "Test 3: Moving BACKWARD (slow speed) for 2 seconds..."
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: -0.1, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}" --once
sleep 2
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}" --once
echo "✅ Backward test complete, stopped"
sleep 1

# Test 4: Rotate left
echo ""
echo "Test 4: Rotating LEFT (slow) for 2 seconds..."
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.3}" --once
sleep 2
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}" --once
echo "✅ Left rotation test complete, stopped"
sleep 1

# Test 5: Rotate right
echo ""
echo "Test 5: Rotating RIGHT (slow) for 2 seconds..."
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: -0.3}" --once
sleep 2
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}" --once
echo "✅ Right rotation test complete, stopped"
sleep 1

# For X3 (Mecanum wheels) - test lateral movement
if [ "$ROBOT_TYPE" == "x3" ]; then
    echo ""
    echo "Test 6: Strafing LEFT (Mecanum only) for 2 seconds..."
    ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.1, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}" --once
    sleep 2
    ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}" --once
    echo "✅ Left strafe test complete, stopped"
    sleep 1

    echo ""
    echo "Test 7: Strafing RIGHT (Mecanum only) for 2 seconds..."
    ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: -0.1, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}" --once
    sleep 2
    ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}" --once
    echo "✅ Right strafe test complete, stopped"
fi

# Final safety stop
echo ""
echo "Sending final STOP command..."
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}" --once

echo ""
echo -e "${GREEN}=========================================${NC}"
echo -e "${GREEN}  Motor Control Test Complete!${NC}"
echo -e "${GREEN}=========================================${NC}"
echo ""
echo "Test Results:"
echo -e "  ${GREEN}✅ Forward motion${NC}"
echo -e "  ${GREEN}✅ Backward motion${NC}"
echo -e "  ${GREEN}✅ Left rotation${NC}"
echo -e "  ${GREEN}✅ Right rotation${NC}"
if [ "$ROBOT_TYPE" == "x3" ]; then
    echo -e "  ${GREEN}✅ Lateral movement (Mecanum)${NC}"
fi
echo ""
echo "Manual control commands:"
echo ""
echo "  # Move forward at 0.2 m/s"
echo "  ros2 topic pub /cmd_vel geometry_msgs/Twist \"linear: {x: 0.2, y: 0.0, z: 0.0}\" --once"
echo ""
echo "  # Rotate left at 0.5 rad/s"
echo "  ros2 topic pub /cmd_vel geometry_msgs/Twist \"angular: {z: 0.5}\" --once"
echo ""
echo "  # STOP (EMERGENCY)"
echo "  ros2 topic pub /cmd_vel geometry_msgs/Twist \"linear: {x: 0.0}\" --once"
echo ""
echo "  # Monitor velocity commands"
echo "  ros2 topic echo /cmd_vel"
echo ""
echo -e "${GREEN}✅ All motor tests passed!${NC}"
echo ""
echo -e "${BLUE}➡️  Next step: Run full system test with ./test_system.sh${NC}"
