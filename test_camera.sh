#!/bin/bash

# Test Camera Feed Script
echo "========================================="
echo "TEST 1: Camera Feed Verification"
echo "========================================="
echo ""

cd /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws
source install/setup.bash
export ROS_DOMAIN_ID=28

echo "Checking for camera topics..."
sleep 3

# List image topics
echo ""
echo "Available camera topics:"
ros2 topic list | grep -i camera || echo "No camera topics found yet"

echo ""
# Check if camera is running
DEPTH_TOPIC=$(ros2 topic list | grep "/camera/depth/image_raw")
IR_TOPIC=$(ros2 topic list | grep "/camera/ir/image_raw")

if [ -z "$DEPTH_TOPIC" ]; then
    echo "⚠️  Camera not running. Launching Astra camera..."
    echo ""
    ros2 launch astra_camera astra.launch.xml &
    CAMERA_PID=$!
    echo "Waiting for camera to initialize..."
    sleep 5
else
    echo "✅ Camera is already running"
fi

echo ""
echo "Checking depth camera topic rate..."
timeout 5 ros2 topic hz /camera/depth/image_raw 2>/dev/null || echo "Depth camera topic not publishing yet"

echo ""
echo "Checking IR camera topic rate..."
timeout 5 ros2 topic hz /camera/ir/image_raw 2>/dev/null || echo "IR camera topic not publishing yet"

echo ""
echo "========================================="
echo "Available camera streams:"
echo "  Depth: /camera/depth/image_raw"
echo "  IR:    /camera/ir/image_raw"
echo "  Cloud: /camera/depth/points"
echo ""
echo "To view camera feed, run in another terminal:"
echo "  source ~/yahboomcar_ros2_ws/setup_env.sh"
echo "  ros2 run rqt_image_view rqt_image_view /camera/depth/image_raw"
echo ""
echo "Or for IR camera:"
echo "  ros2 run rqt_image_view rqt_image_view /camera/ir/image_raw"
echo "========================================="
