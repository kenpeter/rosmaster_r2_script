#!/bin/bash
# 3D World Visualization Diagnostics
# Checks all systems required for RTAB-Map SLAM + AI Fusion Vision

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m'  # No Color

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}3D World Visualization Diagnostics${NC}"
echo -e "${CYAN}========================================${NC}"
echo

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
cd ~/yahboomcar_ros2_ws/yahboomcar_ws
source install/setup.bash
export ROS_DOMAIN_ID=28

# ========================================
# [1] Robot Topics
# ========================================
echo -e "${YELLOW}[1] Robot System${NC}"
if timeout 2 ros2 topic list | grep -q "/odom"; then
    echo -e "  ${GREEN}✓ Robot running${NC}"
else
    echo -e "  ${RED}✗ Robot NOT running${NC}"
    echo -e "    ${YELLOW}Run: ./scripts/start_robot.sh${NC}"
    exit 1
fi

# Check odom rate
ODOM_RATE=$(timeout 5 ros2 topic hz /odom --window 10 2>&1 | grep "average rate" | awk '{print $3}')
if [ ! -z "$ODOM_RATE" ]; then
    echo -e "  ${GREEN}✓ Odometry: $ODOM_RATE Hz${NC}"
else
    echo -e "  ${YELLOW}⚠ Odometry: rate unknown${NC}"
fi

# ========================================
# [2] Camera Topics
# ========================================
echo
echo -e "${YELLOW}[2] Camera System${NC}"

# RGB Camera
timeout 3 ros2 topic echo /camera/color/image_raw --once > /dev/null 2>&1
if [ $? -eq 0 ]; then
    RGB_RATE=$(timeout 5 ros2 topic hz /camera/color/image_raw --window 10 2>&1 | grep "average rate" | awk '{print $3}')
    if [ ! -z "$RGB_RATE" ]; then
        echo -e "  ${GREEN}✓ RGB camera: $RGB_RATE Hz${NC}"
    else
        echo -e "  ${GREEN}✓ RGB camera publishing${NC}"
    fi
else
    echo -e "  ${RED}✗ RGB camera NOT publishing${NC}"
fi

# Depth Camera
timeout 3 ros2 topic echo /camera/depth/image_raw --once > /dev/null 2>&1
if [ $? -eq 0 ]; then
    DEPTH_RATE=$(timeout 5 ros2 topic hz /camera/depth/image_raw --window 10 2>&1 | grep "average rate" | awk '{print $3}')
    if [ ! -z "$DEPTH_RATE" ]; then
        echo -e "  ${GREEN}✓ Depth camera: $DEPTH_RATE Hz${NC}"
    else
        echo -e "  ${GREEN}✓ Depth camera publishing${NC}"
    fi
else
    echo -e "  ${RED}✗ Depth camera NOT publishing${NC}"
fi

# ========================================
# [3] Point Cloud Topics
# ========================================
echo
echo -e "${YELLOW}[3] Point Cloud System${NC}"

FOUND_POINTCLOUD=0
for topic in "/camera/depth_registered/points" "/camera/depth/color/points" "/camera/depth/points"; do
    if timeout 2 ros2 topic list | grep -q "$topic"; then
        timeout 3 ros2 topic echo $topic --once > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo -e "  ${GREEN}✓ Found: $topic (publishing)${NC}"
            FOUND_POINTCLOUD=1
            break
        else
            echo -e "  ${YELLOW}⚠ Found: $topic (exists but no data)${NC}"
        fi
    fi
done

if [ $FOUND_POINTCLOUD -eq 0 ]; then
    echo -e "  ${RED}✗ No point cloud topic publishing${NC}"
    echo -e "    ${YELLOW}Check camera driver: enable_point_cloud: true${NC}"
fi

# ========================================
# [4] LiDAR
# ========================================
echo
echo -e "${YELLOW}[4] LiDAR System${NC}"

timeout 3 ros2 topic echo /scan --once > /dev/null 2>&1
if [ $? -eq 0 ]; then
    LIDAR_RATE=$(timeout 5 ros2 topic hz /scan --window 10 2>&1 | grep "average rate" | awk '{print $3}')
    if [ ! -z "$LIDAR_RATE" ]; then
        echo -e "  ${GREEN}✓ LiDAR: $LIDAR_RATE Hz${NC}"
    else
        echo -e "  ${GREEN}✓ LiDAR publishing${NC}"
    fi
else
    echo -e "  ${RED}✗ LiDAR NOT publishing${NC}"
fi

# ========================================
# [5] RTAB-Map Status
# ========================================
echo
echo -e "${YELLOW}[5] RTAB-Map SLAM${NC}"

if timeout 2 ros2 node list | grep -q "rtabmap"; then
    echo -e "  ${GREEN}✓ RTAB-Map node running${NC}"

    # Check topics
    for topic in "/mapData" "/mapGraph" "/info"; do
        timeout 3 ros2 topic echo $topic --once > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo -e "  ${GREEN}✓ $topic publishing${NC}"
        else
            echo -e "  ${YELLOW}⚠ $topic not publishing${NC}"
        fi
    done
else
    echo -e "  ${YELLOW}⚠ RTAB-Map NOT running${NC}"
    echo -e "    ${YELLOW}Start with: ./scripts/show_3d_world.py${NC}"
fi

# ========================================
# [6] AI Fusion Vision
# ========================================
echo
echo -e "${YELLOW}[6] AI Fusion Vision${NC}"

if timeout 2 ros2 topic list | grep -q "/fusion_vision/output"; then
    timeout 3 ros2 topic echo /fusion_vision/output --once > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        FUSION_RATE=$(timeout 5 ros2 topic hz /fusion_vision/output --window 10 2>&1 | grep "average rate" | awk '{print $3}')
        if [ ! -z "$FUSION_RATE" ]; then
            echo -e "  ${GREEN}✓ Fusion vision: $FUSION_RATE Hz${NC}"
        else
            echo -e "  ${GREEN}✓ Fusion vision publishing${NC}"
        fi
    else
        echo -e "  ${YELLOW}⚠ Topic exists but no data${NC}"
    fi
else
    echo -e "  ${YELLOW}⚠ Fusion vision not running${NC}"
    echo -e "    ${YELLOW}Started by: ./scripts/show_3d_world.py${NC}"
fi

# ========================================
# [7] RViz Configuration
# ========================================
echo
echo -e "${YELLOW}[7] RViz Configuration${NC}"

RVIZ_CONFIG="$HOME/yahboomcar_ros2_ws/yahboomcar_ws/scripts/rtabmap_slam_fusion.rviz"
if [ -f "$RVIZ_CONFIG" ]; then
    echo -e "  ${GREEN}✓ Config file exists${NC}"
    echo -e "    Path: $RVIZ_CONFIG"
else
    echo -e "  ${RED}✗ Config file MISSING${NC}"
    echo -e "    Expected: $RVIZ_CONFIG"
fi

# ========================================
# [8] Active Nodes
# ========================================
echo
echo -e "${YELLOW}[8] Active ROS2 Nodes${NC}"

NODE_COUNT=$(timeout 2 ros2 node list 2>/dev/null | wc -l)
echo -e "  ${CYAN}Total nodes: $NODE_COUNT${NC}"

# Check for critical nodes
CRITICAL_NODES=("/rtabmap" "/camera/camera" "/my_ydlidar_ros2_driver_node" "/ekf_filter_node")
for node in "${CRITICAL_NODES[@]}"; do
    if timeout 2 ros2 node list | grep -q "$node"; then
        echo -e "  ${GREEN}✓ $node${NC}"
    fi
done

# ========================================
# Summary
# ========================================
echo
echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}Diagnostics Complete${NC}"
echo -e "${CYAN}========================================${NC}"
echo

echo -e "${CYAN}Quick Actions:${NC}"
echo -e "  ${YELLOW}• Start robot:${NC}      ./scripts/start_robot.sh"
echo -e "  ${YELLOW}• Start 3D viewer:${NC}  ./scripts/show_3d_world.py"
echo -e "  ${YELLOW}• Topic list:${NC}       ros2 topic list"
echo -e "  ${YELLOW}• Topic monitor:${NC}    ros2 topic echo /mapData"
echo
