#!/bin/bash
# Motor Control Test Script (Continuous Publishing)
# Tests motor control via /cmd_vel topic with CONTINUOUS messages

echo "========================================="
echo "  Motor Control Test (Continuous)"
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
echo ""
read -p "Press ENTER to continue or Ctrl+C to abort..."

# Source ROS2 environment
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"
source "$WORKSPACE_ROOT/install/setup.bash"

echo ""
echo "Checking if /cmd_vel topic exists..."
if ! ros2 topic list | grep -q "^/cmd_vel$"; then
    echo -e "${RED}❌ /cmd_vel topic NOT found!${NC}"
    echo "Make sure robot hardware is running:"
    echo "  ./scripts/start_robot.sh"
    exit 1
fi
echo -e "${GREEN}✅ /cmd_vel topic found!${NC}"

echo ""
echo "========================================="
echo "  Motor Movement Tests (CONTINUOUS)"
echo "========================================="
echo ""

# Test 1: Forward for 3 seconds
echo -e "${BLUE}Test 1: Moving FORWARD at 0.2 m/s for 3 seconds...${NC}"
echo "Press Ctrl+C if robot moves unexpectedly!"
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.2, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}" -r 10 &
PUB_PID=$!
sleep 3
kill $PUB_PID 2>/dev/null
wait $PUB_PID 2>/dev/null

# Stop
echo "Stopping..."
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}" --once
sleep 1

# Test 2: Rotate left for 3 seconds
echo ""
echo -e "${BLUE}Test 2: Rotating LEFT at 0.3 rad/s for 3 seconds...${NC}"
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.3}" -r 10 &
PUB_PID=$!
sleep 3
kill $PUB_PID 2>/dev/null
wait $PUB_PID 2>/dev/null

# Stop
echo "Stopping..."
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}" --once

echo ""
echo -e "${GREEN}=========================================${NC}"
echo -e "${GREEN}  Test Complete!${NC}"
echo -e "${GREEN}=========================================${NC}"
echo ""
echo "If the wheels DID NOT move, check:"
echo "  1. Is the robot driver running? (./scripts/start_robot.sh)"
echo "  2. Is the robot powered on?"
echo "  3. Check driver node: ros2 node list | grep driver"
echo "  4. Check subscriptions: ros2 topic info /cmd_vel"
echo ""
