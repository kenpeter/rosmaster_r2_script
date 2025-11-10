#!/bin/bash
# Stop Robot - User-friendly wrapper for cleanup

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================="
echo "  Stopping Yahboom Robot"
echo "========================================="
echo ""

# Run cleanup script
"$SCRIPT_DIR/cleanup_robot.sh"

echo ""
echo "Robot stopped successfully."
echo ""
echo "To restart the robot, run:"
echo "  ~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/start_robot.sh"
echo ""
