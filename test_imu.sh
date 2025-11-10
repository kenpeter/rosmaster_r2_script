#!/bin/bash
# IMU Test Script
# Tests the /imu/data_raw topic for IMU data

echo "========================================="
echo "  IMU Test Script"
echo "========================================="
echo ""

# Source ROS2 environment
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"
source "$WORKSPACE_ROOT/install/setup.bash"

echo "Testing IMU on topic: /imu/data_raw"
echo ""

# Check if topic exists
echo "Step 1: Checking if /imu/data_raw topic exists..."
if ros2 topic list | grep -q "^/imu/data_raw$"; then
    echo "✅ /imu/data_raw topic found!"
else
    echo "❌ /imu/data_raw topic NOT found!"
    echo ""
    echo "Available topics:"
    ros2 topic list | grep -i imu
    echo ""
    echo "Make sure robot hardware is running:"
    echo "  ros2 launch yahboomcar_bringup yahboomcar_bringup_R2_launch.py  # or X3"
    echo "  or"
    echo "  ros2 launch yahboomcar_bringup yahboomcar_bringup_R2_launch.py"
    exit 1
fi

echo ""
echo "Step 2: Checking topic publishing rate..."
timeout 5 ros2 topic hz /imu/data_raw 2>&1 | head -20 || {
    echo "⚠️  Could not determine rate (topic may not be publishing)"
}

echo ""
echo "Step 3: Getting IMU topic info..."
ros2 topic info /imu/data_raw --verbose

echo ""
echo "Step 4: Reading one IMU sample..."
echo "(This shows orientation, angular velocity, linear acceleration)"
echo ""
timeout 5 ros2 topic echo /imu/data_raw --once || {
    echo "❌ Could not read IMU data"
    exit 1
}

echo ""
echo "========================================="
echo "  IMU Test - Movement Detection"
echo "========================================="
echo ""
echo "Try moving/rotating the robot and watch IMU values change:"
echo ""
echo "To monitor continuously:"
echo "  ros2 topic echo /imu/data_raw"
echo ""
echo "To monitor specific fields:"
echo "  ros2 topic echo /imu/data_raw --field orientation"
echo "  ros2 topic echo /imu/data_raw --field angular_velocity"
echo "  ros2 topic echo /imu/data_raw --field linear_acceleration"
echo ""
echo "To check statistics:"
echo "  ros2 topic hz /imu/data_raw      # Publishing rate"
echo "  ros2 topic bw /imu/data_raw      # Bandwidth"
echo ""
echo "Expected values when stationary:"
echo "  - Angular velocity: ~0.0 rad/s (all axes)"
echo "  - Linear acceleration Z: ~9.81 m/s² (gravity)"
echo "  - Linear acceleration X,Y: ~0.0 m/s²"
echo ""
echo "✅ IMU test complete!"
