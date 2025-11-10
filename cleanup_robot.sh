#!/bin/bash
# Comprehensive Robot Cleanup Script
# Stops all robot processes and cleans up resources

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=========================================${NC}"
echo -e "${BLUE}  Robot System Cleanup${NC}"
echo -e "${BLUE}=========================================${NC}"
echo ""

# Function to kill process by name
kill_process() {
    local process_name=$1
    local display_name=$2

    if pgrep -f "$process_name" > /dev/null; then
        echo -e "${YELLOW}Stopping $display_name...${NC}"
        pkill -f "$process_name"
        sleep 0.5

        # Force kill if still running
        if pgrep -f "$process_name" > /dev/null; then
            echo -e "${YELLOW}Force stopping $display_name...${NC}"
            pkill -9 -f "$process_name"
            sleep 0.5
        fi

        if ! pgrep -f "$process_name" > /dev/null; then
            echo -e "${GREEN}✅ $display_name stopped${NC}"
        else
            echo -e "${RED}⚠️  Failed to stop $display_name${NC}"
        fi
    fi
}

echo "Stopping all robot processes..."
echo ""

# Stop ROS2 launch processes
kill_process "yahboomcar_bringup" "Robot bringup"
kill_process "yahboomcar_autonomous" "Autonomous system"
kill_process "yahboomcar_nav" "Navigation"

# Stop individual robot nodes
kill_process "Ackman_driver_R2" "R2 Driver"
kill_process "Mcnamu_driver_X3" "X3 Driver"
kill_process "base_node_R2" "R2 Base node"
kill_process "base_node_X3" "X3 Base node"
kill_process "driver_node" "Driver node"
kill_process "base_node" "Base node"

# Stop sensors
kill_process "sllidar_node" "LiDAR (SLLIDAR)"
kill_process "rplidar_node" "LiDAR (RPLiDAR)"
kill_process "astra_camera_node" "Astra Camera"
kill_process "imu_filter_madgwick" "IMU filter"

# Stop control nodes
kill_process "yahboom_joy" "Joystick control"
kill_process "joy_node" "Joy node"

# Stop localization
kill_process "ekf_filter_node" "EKF filter"
kill_process "robot_localization" "Robot localization"

# Stop state publishers
kill_process "robot_state_publisher" "Robot state publisher"
kill_process "joint_state_publisher" "Joint state publisher"

# Stop YOLO and vision
kill_process "yolo_ros" "YOLO detection"
kill_process "vlm_scene" "VLM scene understanding"

# Note: Don't kill generic "ros2" processes as it might kill the parent script

echo ""
echo -e "${BLUE}Cleaning up system resources...${NC}"

# Kill any zombie processes
killall -q zombie 2>/dev/null || true

# Clear shared memory (if needed)
# rm -f /dev/shm/sem.* 2>/dev/null || true

# Reset USB devices if needed (optional)
# This can help with stuck camera/lidar
# sudo usbreset 2>/dev/null || true

echo ""
echo -e "${GREEN}=========================================${NC}"
echo -e "${GREEN}  Cleanup Complete!${NC}"
echo -e "${GREEN}=========================================${NC}"

# Check if anything is still running
echo ""
echo "Checking for remaining robot processes..."
REMAINING=$(pgrep -f "yahboomcar\|sllidar\|astra_camera\|Ackman_driver\|base_node" | wc -l)

if [ $REMAINING -eq 0 ]; then
    echo -e "${GREEN}✅ All robot processes stopped${NC}"
else
    echo -e "${YELLOW}⚠️  Warning: $REMAINING robot processes still running${NC}"
    echo ""
    echo "Remaining processes:"
    ps aux | grep -E "yahboomcar|sllidar|astra_camera|Ackman_driver|base_node" | grep -v grep
fi

echo ""
