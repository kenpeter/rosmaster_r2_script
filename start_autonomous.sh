#!/bin/bash
# Start Autonomous Driving System

echo "========================================="
echo "Starting Autonomous Driving System"
echo "========================================="

# Setup environment
source /opt/ros/humble/setup.bash
cd /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws
source install/setup.bash
export ROS_DOMAIN_ID=28

echo "✅ Environment loaded"
echo "🧠 Launching autonomous driving..."
echo ""
echo "This will start:"
echo "  - Lane Detection (OpenCV)"
echo "  - YOLO Object Detection"
echo "  - VLM Scene Understanding (LLaVA)"
echo "  - LLM Decision Making (LLaMA 3)"
echo "  - Control Executor (DISABLED by default)"
echo ""
echo "⚠️  Autonomous mode is DISABLED for safety"
echo "    Robot will NOT move until you enable it"
echo ""

# Launch autonomous system
ros2 launch yahboomcar_autonomous autonomous_driving.launch.py
