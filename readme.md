# YDLidar TG30 Configuration - Fixed and Working! ✅

## Summary
Your YDLidar TG30 is now fully configured and working with ROS2 Humble.

## What Was Fixed

### 1. Device Configuration Issues
**Problem:** Wrong lidar type and channel configuration
- ❌ `lidar_type: 1` (TYPE_TRIANGLE - incorrect)
- ✅ `lidar_type: 0` (TYPE_TOF - correct for TG30)
- ❌ `isSingleChannel: true` (wrong)
- ✅ `isSingleChannel: false` (TG30 is dual-channel)

### 2. Code Issues
**Problem:** GS LiDAR-specific code interfering with TOF lidar
- **File:** `/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/my_ydlidar_ros2_driver/src/my_ydlidar_ros2_driver_node.cpp`
- **Fix:** Commented out `setWorkMode()` calls (lines 165-180)

### 3. Device Identification (Critical!)
**Problem:** Wrong USB device symlink
- ❌ `/dev/ydlidar` was pointing to `/dev/ttyUSB2` (wrong device)
- ✅ `/dev/ydlidar` now points to `/dev/ttyUSB1` (correct device - Silicon Labs CP2102)

## Permanent udev Rule Created ⭐

**File:** `/etc/udev/rules.d/99-ydlidar.rules`

```bash
# YDLidar TG30 udev rule
# This creates a persistent /dev/ydlidar symlink for the YDLidar TG30
# Silicon Labs CP2102 USB-to-UART Bridge (Serial: 0001)
SUBSYSTEM=="tty", ENV{ID_VENDOR_ID}=="10c4", ENV{ID_MODEL_ID}=="ea60", ENV{ID_SERIAL_SHORT}=="0001", SYMLINK+="ydlidar", MODE="0666", GROUP="dialout"
```

**This rule ensures `/dev/ydlidar` always points to your TG30 lidar, even after:**
- System reboots
- USB device reordering
- Plugging into different USB ports

## Your Lidar Specifications

- **Model:** TG30
- **Model Code:** 101
- **Serial:** 2025041600100063
- **Firmware:** 1.71
- **Hardware:** Version 1
- **Baudrate:** 512000
- **Sample Rate:** 20K
- **Scan Frequency:** 10Hz
- **USB Adapter:** Silicon Labs CP2102 (Serial: 0001)

## How to Start Your Lidar

```bash
cd /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws
source install/setup.bash
ros2 launch my_ydlidar_ros2_driver ydlidar_launch.py
```

## Expected Output (Success)

```
[info] Lidar successfully connected [/dev/ydlidar:512000]
[info] Lidar running correctly! The health status good
[info] Current Lidar Model Code 101
[info] Model: TG30
[info] Serial: 2025041600100063
[info] Sample Rate: 20.00K
[info] Scan Frequency: 10.00Hz
[info] Now lidar is scanning...
```

## Configuration Files

1. **Parameters:** `/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/my_ydlidar_ros2_driver/params/ydlidar.yaml`
   - `lidar_type: 0` (TYPE_TOF)
   - `isSingleChannel: false`
   - `baudrate: 512000`
   - `sample_rate: 20`
   - `frequency: 10.0`

2. **Driver:** `/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/my_ydlidar_ros2_driver/src/my_ydlidar_ros2_driver_node.cpp`
   - GS LiDAR `setWorkMode()` calls disabled (lines 165-180)

3. **udev Rule:** `/etc/udev/rules.d/99-ydlidar.rules`
   - Permanent `/dev/ydlidar` symlink based on USB serial number

## Troubleshooting

### If the symlink is missing after reboot:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
ls -l /dev/ydlidar  # Should show: /dev/ydlidar -> ttyUSB1
```

### If you get "Device Tremble" or timeout errors:
1. Check the symlink points to the correct device:
   ```bash
   ls -l /dev/ydlidar
   ```
2. Verify it points to the Silicon Labs CP2102 device:
   ```bash
   udevadm info /dev/ydlidar | grep ID_SERIAL
   # Should show: Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001
   ```

### If lidar is on a different USB port:
The udev rule automatically creates the symlink based on the USB chip's unique serial number (`0001`), not the port number. Your lidar will always be `/dev/ydlidar` regardless of which USB port you plug it into! 🎯

## Testing

To verify scan data is being published:
```bash
# In terminal 1:
ros2 launch my_ydlidar_ros2_driver ydlidar_launch.py

# In terminal 2:
ros2 topic list | grep scan
ros2 topic hz /scan
ros2 topic echo /scan --once
```

## Summary of Changes

### Files Modified:
1. `/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/my_ydlidar_ros2_driver/params/ydlidar.yaml`
2. `/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/my_ydlidar_ros2_driver/src/my_ydlidar_ros2_driver_node.cpp`

### Files Created:
1. `/etc/udev/rules.d/99-ydlidar.rules` (permanent udev rule)

### Package Rebuilt:
```bash
cd /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws
colcon build --packages-select my_ydlidar_ros2_driver
```

## Success! 🎉

Your YDLidar TG30 is now properly configured with a **permanent udev rule** and will work reliably every time you start it!

**The device will always be available as `/dev/ydlidar` regardless of:**
- System reboots
- USB port changes
- Other USB devices plugged in

---

**Date Fixed:** 2025-12-11
**Fixed By:** Claude Code
**Driver Version:** 1.0.0
**SDK Version:** 1.2.1






  Start the lidar:
  cd /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws
  source install/setup.bash
  ros2 launch my_ydlidar_ros2_driver ydlidar_launch.py

  Check scan data (in another terminal):
  cd /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws
  source install/setup.bash
  ros2 topic list
  ros2 topic hz /scan
  ros2 topic echo /scan --once



  # If the driver is not running, start it first in Terminal 1:
  cd /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws
  source install/setup.bash
  ros2 launch my_ydlidar_ros2_driver ydlidar_launch.py

  # In Terminal 2, restart RViz2:
  cd /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws
  source install/setup.bash
  rviz2 -d src/yahboomcar_rviz/rviz/laser_scan.rviz


