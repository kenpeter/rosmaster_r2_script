# Yahboom Rosmaster R2 - Robot Scripts

Simplified script collection: **7 scripts** (reduced from 28)

## Quick Start

```bash
# 1. Diagnose hardware
./diagnose_robot.py --quick

# 2. Auto-fix issues (if needed)
./diagnose_robot.py --fix

# 3. Start robot
./start_robot.sh

# 4. Stop robot
./stop_robot.sh
```

## All Scripts

### Core Operations
- **`diagnose_robot.py`** - All-in-one diagnostic (replaces 20+ test scripts)
- **`start_robot.sh`** - Launch robot system
- **`stop_robot.sh`** - Stop all processes

### Utilities
- `cleanup_robot.sh` - Clean up resources
- `wifi-switch.sh` - WiFi management
- `setup_env.sh` - ROS2 environment setup
- `start_autonomous.sh` - Launch autonomous navigation

## Diagnose Robot Tool

### Options
```bash
./diagnose_robot.py --quick    # Quick health check
./diagnose_robot.py            # Full diagnostic + ROS2
./diagnose_robot.py --fix      # Auto-fix common issues
```

### What it tests
- ✅ USB devices (ROSMaster, LiDAR, Camera)
- ✅ ROSMaster board communication
- ✅ YDLiDAR motor control & scan data
- ✅ ROS2 topics (full diagnostic only)

### Auto-fix features
- Dialout group membership
- USB permissions
- Interfering processes

## Hardware Ports

| Component | Port | Baud | Topics |
|-----------|------|------|--------|
| ROSMaster | `/dev/ttyUSB0` | 115200 | Motors, IMU, voltage |
| YDLiDAR A1 | `/dev/ydlidar` | 512000 | `/scan` |
| Astra Camera | USB | - | `/camera/depth/*` |

## Troubleshooting

### LiDAR not working
```bash
# 1. Check motor spinning (listen for whirr)
# 2. Run diagnostic
./diagnose_robot.py

# 3. Check physical connections
ls -l /dev/ydlidar
```

**Common issue**: Motor not spinning
- Some models need external 5V power
- DTR motor control may not work
- Check power LED on LiDAR unit

### Motors not responding
```bash
# 1. Run auto-fix
./diagnose_robot.py --fix

# 2. Restart system
./stop_robot.sh
./start_robot.sh

# 3. Check voltage
ros2 topic echo /voltage --once
```

### Camera not detected
```bash
# Check USB
lsusb | grep 2bc5

# Restart robot
./stop_robot.sh && ./start_robot.sh
```

## Removed Scripts

The following **21 scripts** have been merged into `diagnose_robot.py`:
- All `test_*.sh` and `test_*.py`
- All `check_*.sh`
- `comprehensive_rosmaster_test.py`
- `diagnose_ydlidar.py`
- `find_robot_usb.sh`
- `find_rosmaster_port.py`
- `reset_rosmaster.py`
- `fix_and_restart.sh`
- And 13 others...

## Support

1. Run `./diagnose_robot.py` for automated diagnostics
2. Check ROS2 logs: `~/.ros/log/`
3. Yahboom support: support@yahboom.com

---

**Created**: November 2025
**Workspace**: `~/yahboomcar_ros2_ws/yahboomcar_ws`
**ROS2**: Humble
