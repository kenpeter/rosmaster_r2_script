#!/bin/bash
# LiDAR Test Script
# Tests the /scan topic for LiDAR data

echo "========================================="
echo "  LiDAR Test Script"
echo "========================================="
echo ""

# Source ROS2 environment
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"
source "$WORKSPACE_ROOT/install/setup.bash"

echo "Testing LiDAR on topic: /scan"
echo ""

# Check if topic exists
echo "Step 1: Checking if /scan topic exists..."
if ros2 topic list | grep -q "^/scan$"; then
    echo "✅ /scan topic found!"
else
    echo "❌ /scan topic NOT found!"
    echo ""
    echo "Available topics:"
    ros2 topic list
    echo ""
    echo "Make sure robot hardware is running:"
    echo "  ros2 launch yahboomcar_bringup yahboomcar_bringup_R2_launch.py  # or X3"
    echo "  or"
    echo "  ros2 launch yahboomcar_bringup yahboomcar_bringup_R2_launch.py"
    exit 1
fi

echo ""
echo "Step 2: Checking topic publishing rate..."
timeout 5 ros2 topic hz /scan 2>&1 | head -20 || {
    echo "⚠️  Could not determine rate (topic may not be publishing)"
}

echo ""
echo "Step 3: Getting LiDAR info..."
ros2 topic info /scan --verbose

echo ""
echo "Step 4: Reading one LiDAR scan..."
echo "(This shows range data in meters, angle info, etc.)"
echo ""
timeout 5 ros2 topic echo /scan --once || {
    echo "❌ Could not read LiDAR data"
    exit 1
}

echo ""
echo "========================================="
echo "  LiDAR Test Options"
echo "========================================="
echo ""
echo "To visualize LiDAR data in RViz:"
echo "  rviz2"
echo "  Then: Add -> LaserScan -> Topic: /scan"
echo "  Set Fixed Frame to: base_link or laser_frame"
echo ""
echo "To monitor continuously:"
echo "  ros2 topic echo /scan"
echo ""
echo "To check statistics:"
echo "  ros2 topic hz /scan      # Publishing rate"
echo "  ros2 topic bw /scan      # Bandwidth"
echo ""
echo "✅ LiDAR test complete!"
