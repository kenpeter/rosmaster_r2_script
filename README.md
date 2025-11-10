# Yahboom Robot Scripts

This directory contains utility scripts for testing and running the Yahboom robot system.

## 🧪 Test Scripts (Organized by Type)

### Quick Test Guide

```bash
# Step 1: Start robot hardware (R2 - Ackermann steering)
ros2 launch yahboomcar_bringup yahboomcar_bringup_R2_launch.py

# Step 2: Test sensors (non-moving)
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/test_sensors.sh

# Step 3: Test motors (moving - requires clear space!)
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/test_motor.sh

# Step 4: Full system integration test
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/test_system.sh
```

---

## Primary Test Scripts

### 1. `test_sensors.sh` ✅ (NON-MOVING)
**Tests all sensors WITHOUT moving the robot**

**Tests included:**
- ✅ Camera feed (`/RGBD/RGB/Image`)
- ✅ LiDAR sensor (`/scan`)
- ✅ IMU sensor (`/imu/data_raw`)

**Usage:**
```bash
# Make sure robot hardware is running first
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/test_sensors.sh
```

**Safety:** ✅ Robot will NOT move
**Duration:** ~30 seconds
**Prerequisites:** Robot hardware must be running

**Output:**
- Checks topic availability
- Verifies publishing rates
- Validates data format
- Provides visualization commands

---

### 2. `test_motor.sh` ⚠️ (MOVING TEST)
**Tests motor control - ROBOT WILL MOVE!**

**Tests included:**
- ✅ Forward motion
- ✅ Backward motion
- ✅ Left rotation
- ✅ Right rotation
- ⊘ Lateral movement (Mecanum/X3 only - not supported on R2 Ackermann)

**Usage:**
```bash
# ⚠️ WARNING: Ensure clear space (1+ meter on all sides)
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/test_motor.sh
```

**Safety:** ⚠️ Robot WILL move
**Duration:** ~15 seconds
**Prerequisites:**
- Robot hardware must be running
- Clear space around robot
- Human supervision

**Movement parameters:**
- Linear speed: 0.1 m/s (slow/safe)
- Angular speed: 0.3 rad/s (slow rotation)
- Each movement: 1-2 seconds
- Auto-stop after each test

---

### 3. `test_system.sh` 🚀 (COMPREHENSIVE)
**Complete system integration test including autonomous capabilities**

**Three-phase testing:**

**Phase 1: Sensors (non-moving)**
- Camera, LiDAR, IMU verification

**Phase 2: Motors (moving)**
- Basic movement validation
- ⚠️ Robot will move briefly

**Phase 3: Autonomous (optional)**
- Lane detection
- YOLO object detection
- VLM scene understanding
- LLM decision making

**Usage:**
```bash
# Full integration test
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/test_system.sh
```

**Safety:** ⚠️ Robot will move in Phase 2
**Duration:** 2-5 minutes (depending on options)
**Prerequisites:**
- Robot hardware running
- (Optional) Autonomous system running for Phase 3

**Interactive prompts:**
- Confirms before moving robot
- Option to skip autonomous tests
- Step-by-step progression

---

## Individual Component Tests (Advanced)

### `test_camera.sh`
Camera-only test with detailed output.

### `test_lidar.sh`
LiDAR-only test with RViz visualization commands.

### `test_imu.sh`
IMU-only test showing orientation and acceleration data.

### `test_hardware.sh` (Legacy)
Original combined hardware test (use `test_system.sh` instead).

---

## Setup Scripts

### `setup_env.sh`
Sets up the ROS2 environment for the yahboomcar workspace.

**Usage:**
```bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/setup_env.sh
```

**What it does:**
- Sources ROS2 Humble
- Sources library workspace overlay
- Sources yahboomcar workspace overlay
- Sets ROS_DOMAIN_ID=28
- Displays environment info

**Note:** This is automatically called by ~/.bashrc

---

## Launch Scripts

### `start_robot.sh`
Quick launcher for robot hardware (bringup).

**Usage:**
```bash
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/start_robot.sh
```

### `start_autonomous.sh`
Quick launcher for autonomous driving system.

**Usage:**
```bash
# Robot hardware must already be running
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/start_autonomous.sh
```

---

## Utility Scripts

### `wifi-switch.sh`
WiFi network management utility.

---

## Testing Workflow

### For First-Time Setup

```bash
# Terminal 1: Start robot (R2 - Ackermann steering)
cd ~/yahboomcar_ros2_ws/yahboomcar_ws
source scripts/setup_env.sh
ros2 launch yahboomcar_bringup yahboomcar_bringup_R2_launch.py

# Terminal 2: Run progressive tests
# Step 1: Non-moving sensor test
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/test_sensors.sh

# Step 2: Moving motor test (ensure clear space!)
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/test_motor.sh

# Step 3: Full system test
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/test_system.sh
```

### For Quick Verification

```bash
# Terminal 1: Robot hardware (R2)
ros2 launch yahboomcar_bringup yahboomcar_bringup_R2_launch.py

# Terminal 2: Quick check
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/test_sensors.sh
```

### For Autonomous Development

```bash
# Terminal 1: Robot hardware (R2)
ros2 launch yahboomcar_bringup yahboomcar_bringup_R2_launch.py

# Terminal 2: Autonomous system
ros2 launch yahboomcar_autonomous autonomous_driving.launch.py

# Terminal 3: Full system test
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/test_system.sh
```

---

## Test Script Comparison

| Script | Movement | Duration | Purpose |
|--------|----------|----------|---------|
| `test_sensors.sh` | ❌ No | ~30s | Verify sensor hardware |
| `test_motor.sh` | ✅ Yes | ~15s | Verify motor control |
| `test_system.sh` | ⚠️ Phase 2 | 2-5min | Complete integration |
| `test_camera.sh` | ❌ No | ~10s | Camera only (detailed) |
| `test_lidar.sh` | ❌ No | ~10s | LiDAR only (detailed) |
| `test_imu.sh` | ❌ No | ~10s | IMU only (detailed) |

---

## Safety Guidelines

### Before Running Motor Tests

- [ ] Clear space: 1+ meter on all sides
- [ ] Robot on flat surface
- [ ] No people/pets nearby
- [ ] No obstacles or fragile items
- [ ] Emergency stop ready (Ctrl+C)
- [ ] Human supervision at all times

### Emergency Stop

**During test:**
- Press `Ctrl+C` in terminal

**Manual stop command:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0}" --once
```

---

## Troubleshooting

### "Topic not found" errors

**Solution:**
```bash
# Make sure robot hardware is running
ros2 node list

# Should show nodes like:
# /yahboomcar_driver
# /astra_camera
# /rplidar_node
```

### Sensor not publishing

**Check:**
```bash
# View all topics
ros2 topic list

# Check specific topic rate
ros2 topic hz /scan
ros2 topic hz /RGBD/RGB/Image
ros2 topic hz /imu/data_raw
```

### Motor control not working

**Verify:**
```bash
# Check if /cmd_vel is subscribed
ros2 topic info /cmd_vel

# Manually send test command
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.1}" --once

# Emergency stop
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0}" --once
```

### Test script won't run

**Solution:**
```bash
# Make sure scripts are executable
chmod +x ~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/*.sh

# Or run with bash
bash ~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/test_sensors.sh
```

---

## Script Maintenance

All test scripts in this directory:
- ✅ Are executable (`chmod +x`)
- ✅ Auto-detect workspace location
- ✅ Source ROS2 environment automatically
- ✅ Use color-coded output
- ✅ Provide safety warnings (when applicable)
- ✅ Include usage examples
- ✅ Give helpful error messages

### Script Template

To add a new test script, use this template:

```bash
#!/bin/bash
# Script description

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Source ROS2 environment
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"
source "$WORKSPACE_ROOT/install/setup.bash"

# Your test code here
echo -e "${GREEN}✅ Test passed${NC}"
```

---

## Quick Reference Commands

### Manual Testing

```bash
# View camera
ros2 run rqt_image_view rqt_image_view /RGBD/RGB/Image

# View LiDAR in RViz
rviz2

# Monitor IMU
ros2 topic echo /imu/data_raw

# Check all topics
ros2 topic list

# Check topic rates
ros2 topic hz <topic_name>

# Send motor command
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.2}" --once

# Emergency stop
ros2 topic pub /cmd_vel geometry_msgs/Twist "{}" --once
```

---

## Related Documentation

- [Quick Start Guide](../docs/QUICK_START_GUIDE.md)
- [Testing Procedure](../docs/TESTING_PROCEDURE.md)
- [LLM Autonomous Driving Guide](../docs/LLM_AUTONOMOUS_DRIVING_GUIDE.md)
- [Self-Driving Components](../docs/SELF_DRIVING_REUSABLE_COMPONENTS.md)

---

**Test Script Organization:**
- **Non-Moving:** `test_sensors.sh` - Safe, quick sensor validation
- **Moving:** `test_motor.sh` - Motor control verification (requires clear space)
- **Integration:** `test_system.sh` - Complete system test (sensors + motors + autonomous)

**Created:** November 10, 2025
**Workspace:** ~/yahboomcar_ros2_ws/yahboomcar_ws
**ROS2 Version:** Humble
