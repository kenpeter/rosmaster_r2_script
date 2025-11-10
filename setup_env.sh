#!/bin/bash
# ROS2 Environment Setup Script

# Get the workspace root (parent of scripts/)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

# Source system ROS2 first
source /opt/ros/humble/setup.bash

# Then source workspace overlay
source "$WORKSPACE_ROOT/install/setup.bash"

# Set ROS domain ID
export ROS_DOMAIN_ID=28

echo "✅ ROS2 Environment Ready!"
echo "   ROS_DISTRO: $ROS_DISTRO"
echo "   ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "   Workspace: $WORKSPACE_ROOT"
echo ""
