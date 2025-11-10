#!/bin/bash
# Fix common issues and restart robot cleanly

echo "========================================="
echo "  Fixing Issues and Restarting Robot"
echo "========================================="
echo ""

# Step 1: Kill all existing robot processes
echo "Step 1: Stopping all robot processes..."
pkill -f "yahboomcar_bringup" 2>/dev/null
pkill -f "sllidar" 2>/dev/null
pkill -f "astra_camera" 2>/dev/null
pkill -f "Ackman_driver" 2>/dev/null
pkill -f "base_node" 2>/dev/null
sleep 2
echo "✅ All processes stopped"
echo ""

# Step 2: Fix LiDAR permissions
echo "Step 2: Fixing LiDAR permissions..."
if [ -e "/dev/ttyUSB0" ]; then
    sudo chmod 666 /dev/ttyUSB0
    echo "✅ LiDAR permissions fixed (/dev/ttyUSB0)"
elif [ -e "/dev/ttyUSB1" ]; then
    sudo chmod 666 /dev/ttyUSB1
    echo "✅ LiDAR permissions fixed (/dev/ttyUSB1)"
else
    echo "⚠️  No /dev/ttyUSB* found - LiDAR may not be connected"
fi
echo ""

# Step 3: Verify devices
echo "Step 3: Checking devices..."
echo "USB devices:"
lsusb | grep -E "FTDI|Silicon|Prolific|rplidar" || echo "  No LiDAR device found"
lsusb | grep -E "Orbbec|Astra" || echo "  No camera device found"
echo ""

# Step 4: Restart robot
echo "Step 4: Starting robot system..."
echo ""
echo "Press ENTER to launch robot, or Ctrl+C to cancel..."
read

# Source ROS2 environment
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"
source "$WORKSPACE_ROOT/install/setup.bash"

# Launch
exec "$SCRIPT_DIR/start_robot.sh"
