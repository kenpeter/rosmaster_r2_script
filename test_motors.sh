#!/bin/bash
###############################################################################
# Motor Test Script
# Tests all motor movements (ROBOT WILL MOVE!)
###############################################################################

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}=========================================${NC}"
echo -e "${BLUE}  Yahboom Motor Test${NC}"
echo -e "${BLUE}=========================================${NC}"
echo ""

# Safety check
echo -e "${RED}⚠️  WARNING: ROBOT WILL MOVE!${NC}"
echo ""
echo "Please ensure:"
echo "  1. Clear space (1+ meter around robot)"
echo "  2. Robot on floor (not on table)"
echo "  3. No obstacles nearby"
echo ""
read -p "Press ENTER to continue or Ctrl+C to cancel..."
echo ""

# Check if robot is running
echo -e "${YELLOW}[1/6] Checking robot status...${NC}"
if ros2 topic info /cmd_vel &>/dev/null; then
    echo -e "${GREEN}✅ Motor control ready${NC}"
else
    echo -e "${RED}❌ Robot not running! Start with: ./start_robot.sh${NC}"
    exit 1
fi

# Function to send movement command
move_robot() {
    local name=$1
    local linear_x=$2
    local angular_z=$3
    local duration=$4

    echo -e "${YELLOW}[Test] $name...${NC}"
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: $linear_x, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $angular_z}}"
    sleep $duration

    # Stop
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    sleep 0.5
    echo -e "${GREEN}✅ $name complete${NC}"
    echo ""
}

# Run tests
echo -e "${BLUE}=========================================${NC}"
echo -e "${BLUE}  Starting Motor Tests${NC}"
echo -e "${BLUE}=========================================${NC}"
echo ""

move_robot "Forward" 0.15 0.0 2
move_robot "Backward" -0.15 0.0 2
move_robot "Turn Left" 0.0 0.3 2
move_robot "Turn Right" 0.0 -0.3 2
move_robot "Forward + Left Turn" 0.1 0.2 2

# Final stop
echo -e "${YELLOW}[6/6] Final stop...${NC}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
echo -e "${GREEN}✅ All motors stopped${NC}"

echo ""
echo -e "${BLUE}=========================================${NC}"
echo -e "${GREEN}  Motor Test Complete!${NC}"
echo -e "${BLUE}=========================================${NC}"
echo ""
echo "Did all movements work?"
echo "  - Forward ✓"
echo "  - Backward ✓"
echo "  - Turn Left ✓"
echo "  - Turn Right ✓"
echo "  - Combined movement ✓"
echo ""
