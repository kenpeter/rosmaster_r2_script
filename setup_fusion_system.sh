#!/bin/bash
# Setup Script for LiDAR-Camera Fusion System
# This script builds and sets up the complete fusion system

set -e  # Exit on error

echo "========================================="
echo "LiDAR-Camera Fusion System Setup"
echo "========================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Navigate to workspace
cd /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws

echo -e "${YELLOW}Step 1: Sourcing ROS2 environment...${NC}"
source /opt/ros/humble/setup.bash

echo -e "${YELLOW}Step 2: Building lidar_camera_fusion package...${NC}"
colcon build --packages-select lidar_camera_fusion

echo -e "${YELLOW}Step 3: Sourcing workspace...${NC}"
source install/setup.bash

echo ""
echo -e "${GREEN}✓ Build completed successfully!${NC}"
echo ""
echo "========================================="
echo "Next Steps:"
echo "========================================="
echo ""
echo "1. CALIBRATE YOUR CAMERA (first time only):"
echo "   ros2 launch lidar_camera_fusion camera_calibration.launch.py"
echo ""
echo "2. SAVE CALIBRATION DATA:"
echo "   ros2 launch lidar_camera_fusion save_calibration.launch.py"
echo ""
echo "3. EDIT EXTRINSIC CALIBRATION:"
echo "   nano ~/yahboomcar_ros2_ws/yahboomcar_ws/src/lidar_camera_fusion/config/my_calibration.yaml"
echo ""
echo "4. RUN THE COMPLETE SYSTEM:"
echo "   ros2 launch lidar_camera_fusion complete_fusion_system.launch.py \\"
echo "     calibration_file:=~/yahboomcar_ros2_ws/yahboomcar_ws/src/lidar_camera_fusion/config/my_calibration.yaml"
echo ""
echo "========================================="
echo "For detailed instructions, see:"
echo "  ~/yahboomcar_ros2_ws/yahboomcar_ws/src/lidar_camera_fusion/QUICK_START.md"
echo "  ~/yahboomcar_ros2_ws/yahboomcar_ws/src/lidar_camera_fusion/README.md"
echo "========================================="
