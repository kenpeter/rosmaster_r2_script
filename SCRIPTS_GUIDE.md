# Robot Scripts Guide

Complete guide to all robot control and test scripts.

---

## 🚀 Quick Start

### Start Robot (Clean Launch)
```bash
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/start_robot.sh
```

**What it does:**
1. ✅ **Auto-cleanup** - Stops all existing processes
2. ✅ **Fix permissions** - Sets LiDAR/camera permissions
3. ✅ **Load environment** - Sources ROS2 workspace
4. ✅ **Launch everything** - Robot + LiDAR + Camera + IMU

### Stop Robot
```bash
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/stop_robot.sh
```

Or just press **Ctrl+C** in the launch terminal.

---

## 📜 Available Scripts

### Core Control Scripts

| Script | Purpose | Auto-Cleanup |
|--------|---------|--------------|
| `start_robot.sh` | Launch full robot system | ✅ Yes |
| `stop_robot.sh` | Stop robot gracefully | N/A |
| `cleanup_robot.sh` | Clean up processes only | N/A |

### Test Scripts

| Script | Type | Movement | Tests |
|--------|------|----------|-------|
| `test_sensors.sh` | Non-moving | ❌ No | Camera, LiDAR, IMU |
| `test_motor.sh` | Moving | ✅ Yes | Forward, backward, rotation |
| `test_system.sh` | Complete | ⚠️ Phase 2 | All sensors + motors + autonomous |

### Individual Component Tests

| Script | Tests |
|--------|-------|
| `test_camera.sh` | Camera only |
| `test_lidar.sh` | LiDAR only |
| `test_imu.sh` | IMU only |

### Other Scripts

| Script | Purpose |
|--------|---------|
| `setup_env.sh` | Environment setup (called by .bashrc) |
| `start_autonomous.sh` | Launch autonomous driving |
| `wifi-switch.sh` | WiFi management |

---

## 🔄 Typical Workflow

### 1. Start Robot
```bash
# This automatically cleans up first!
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/start_robot.sh
```

### 2. Test Sensors (New Terminal)
```bash
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/test_sensors.sh
```

### 3. Test Motors (Ensure clear space!)
```bash
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/test_motor.sh
```

### 4. Stop Robot
```bash
# Press Ctrl+C in robot terminal
# OR run:
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/stop_robot.sh
```

---

## 🧹 Cleanup System

### Automatic Cleanup (Built into start_robot.sh)

Every time you run `start_robot.sh`, it **automatically**:
- ✅ Stops all existing robot processes
- ✅ Kills duplicate nodes
- ✅ Cleans up resources
- ✅ Fixes device permissions
- ✅ Then starts fresh

**No manual cleanup needed!**

### Manual Cleanup

If you need to clean up without starting:
```bash
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/cleanup_robot.sh
```

Or stop everything gracefully:
```bash
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/stop_robot.sh
```

---

## 🛡️ What Gets Cleaned Up

The cleanup system stops all:
- ✅ Robot bringup processes
- ✅ Driver nodes (R2/X3)
- ✅ Base nodes
- ✅ Sensor nodes (LiDAR, Camera, IMU)
- ✅ Localization (EKF)
- ✅ State publishers
- ✅ Joystick control
- ✅ Autonomous system
- ✅ YOLO/VLM vision
- ✅ Any zombie processes

**Result:** Clean state, no duplicates!

---

## 🔍 Troubleshooting

### "Multiple nodes with same name" Warning

**Problem:** Duplicate processes running
**Solution:** Already fixed! `start_robot.sh` now auto-cleans

### LiDAR Not Working

**Problem:** Permission denied
**Solution:** Already fixed! `start_robot.sh` auto-fixes permissions

Or manually:
```bash
sudo chmod 666 /dev/ttyUSB0
```

### Camera Topic Not Found

**Problem:** Using old topic name
**Solution:** Already fixed! `test_sensors.sh` now checks both:
- `/RGBD/RGB/Image` (old)
- `/camera/color/image_raw` (new Astra)

### Process Won't Stop

**Force cleanup:**
```bash
# Run cleanup script
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/cleanup_robot.sh

# Or manually kill all
pkill -9 -f yahboomcar
```

---

## 📊 Script Execution Flow

### start_robot.sh Flow

```
1. Display banner
   ↓
2. Run cleanup_robot.sh
   ├─ Stop all robot processes
   ├─ Kill duplicates
   └─ Clean resources
   ↓
3. Fix device permissions
   ├─ LiDAR (/dev/ttyUSB*)
   └─ Camera (check USB)
   ↓
4. Load ROS2 environment
   ├─ Source ROS2 Humble
   └─ Source workspace
   ↓
5. Launch robot system
   └─ yahboomcar_bringup_R2_full_launch.py
      ├─ Robot hardware
      ├─ LiDAR
      ├─ Camera
      ├─ IMU
      └─ Localization
```

---

## 🎯 Best Practices

### ✅ DO

- **Always use `start_robot.sh`** - It handles cleanup automatically
- **Wait for launch to complete** - Takes ~10 seconds
- **Use test scripts** - Verify everything works
- **Stop cleanly** - Ctrl+C or `stop_robot.sh`

### ❌ DON'T

- **Don't run multiple instances** - `start_robot.sh` prevents this
- **Don't skip cleanup** - Already automatic
- **Don't manually launch** - Use the scripts
- **Don't kill -9 immediately** - Try Ctrl+C first

---

## 📁 Script Locations

All scripts are in:
```
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/
```

**Executable scripts:**
- ✅ All `.sh` files are executable
- ✅ Can run from anywhere
- ✅ Auto-detect workspace location

**Documentation:**
- `README.md` - Detailed script documentation
- `SCRIPTS_GUIDE.md` - This file

---

## 🔗 Related Documentation

- [Launch Guide](../LAUNCH_GUIDE.md) - Launch file documentation
- [Robot Config](../ROBOT_CONFIG.md) - R2 robot specifications
- [Quick Start](../docs/QUICK_START_GUIDE.md) - Getting started
- [Testing Procedure](../docs/TESTING_PROCEDURE.md) - Detailed tests

---

## 📝 Script Maintenance

### Adding a New Script

1. Create in `scripts/` directory
2. Add shebang: `#!/bin/bash`
3. Make executable: `chmod +x script_name.sh`
4. Use workspace auto-detection:
```bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"
source "$WORKSPACE_ROOT/install/setup.bash"
```

### Script Template

```bash
#!/bin/bash
# Script Description

# Get paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

# Source ROS2
source "$WORKSPACE_ROOT/install/setup.bash"

# Your code here
echo "Hello, Robot!"
```

---

## 🎓 Examples

### Example 1: Clean Start Every Time
```bash
# Just run start - cleanup is automatic
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/start_robot.sh
```

### Example 2: Test After Launch
```bash
# Terminal 1: Start robot
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/start_robot.sh

# Terminal 2: Wait 10 seconds, then test
sleep 10
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/test_sensors.sh
```

### Example 3: Stop and Restart
```bash
# Stop
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/stop_robot.sh

# Start fresh (auto-cleanup included)
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/start_robot.sh
```

### Example 4: Manual Cleanup Only
```bash
# Just cleanup, don't start
~/yahboomcar_ros2_ws/yahboomcar_ws/scripts/cleanup_robot.sh
```

---

**Last Updated:** November 10, 2025
**Robot:** Yahboom Rosmaster R2 (Ackermann)
**ROS2:** Humble
**Key Feature:** ✅ Auto-cleanup before every launch!
