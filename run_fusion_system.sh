#!/bin/bash
# Run Script for LiDAR-Camera Fusion System
# This script launches the complete fusion system with all components

set -e  # Exit on error

echo "========================================="
echo "Starting LiDAR-Camera Fusion System"
echo "========================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Navigate to workspace
cd /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws

# Source ROS2 and workspace
echo -e "${YELLOW}Sourcing ROS2 environment...${NC}"
source /opt/ros/humble/setup.bash
source install/setup.bash

# Check if calibration file exists
CALIB_FILE="$HOME/yahboomcar_ros2_ws/yahboomcar_ws/src/lidar_camera_fusion/config/my_calibration.yaml"
DEFAULT_CALIB="$HOME/yahboomcar_ros2_ws/yahboomcar_ws/src/lidar_camera_fusion/config/default_calibration.yaml"

if [ -f "$CALIB_FILE" ]; then
    echo -e "${GREEN}✓ Using calibration file: $CALIB_FILE${NC}"
    CALIBRATION_ARG="calibration_file:=$CALIB_FILE"
else
    echo -e "${YELLOW}⚠ Custom calibration not found, using default calibration${NC}"
    echo -e "${YELLOW}⚠ For accurate results, please calibrate your camera first!${NC}"
    echo ""
    echo "To calibrate, run:"
    echo "  ros2 launch lidar_camera_fusion camera_calibration.launch.py"
    echo ""
    CALIBRATION_ARG="calibration_file:=$DEFAULT_CALIB"
fi

echo ""
echo -e "${GREEN}Starting complete fusion system...${NC}"
echo ""
echo "This will launch:"
echo "  ✓ Robot base (motors, odometry, IMU)"
echo "  ✓ YDLIDAR TG30 driver"
echo "  ✓ Astra camera"
echo "  ✓ SLAM (slam_toolbox)"
echo "  ✓ LiDAR-Camera fusion"
echo "  ✓ RViz visualization"
echo ""

# Launch the system
ros2 launch lidar_camera_fusion complete_fusion_system.launch.py "$CALIBRATION_ARG"
