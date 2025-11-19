# YDLidar X4 Testing Guide

## Hardware Status
- ✅ Serial communication working (`/dev/ydlidar` accessible)
- ✅ Data being received from lidar
- ❌ SDK commands not responding (health check fails)

## Quick Hardware Test

Run this script to verify hardware communication:
```bash
cd ~/yahboomcar_ros2_ws/yahboomcar_ws/scripts
python3 test_ydlidar.py
```

## Common Issues

### Issue 1: Motor Not Spinning
**Symptom**: Lidar doesn't respond to SDK commands
**Check**: Is the motor physically spinning? (you should hear it)
**Solution**:
- Check 5V power connection to lidar
- Red wire should have 5V
- Motor needs power to spin

### Issue 2: Wrong Baudrate
**Status**: ✅ FIXED (changed from 512000 to 128000)

### Issue 3: Wrong Device Path
**Status**: ✅ FIXED (udev rule creates `/dev/ydlidar` symlink)

## ROS2 Testing

After confirming motor is spinning, launch the driver:
```bash
source ~/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

Check for scan data:
```bash
ros2 topic list | grep scan
ros2 topic echo /scan --once
```

## Configuration File
Location: `/home/jetson/yahboomcar_ros2_ws/software/library_ws/src/ydlidar_ros2_driver/params/ydlidar.yaml`

Current settings:
- Port: `/dev/ydlidar`
- Baudrate: `128000` (correct for X4)
- Frame: `laser`
- Frequency: `10.0` Hz

## Next Steps

1. Check if lidar motor is spinning
2. If not spinning, check power connections
3. If spinning, try adjusting these parameters in ydlidar.yaml:
   - `sample_rate`: try 9 or 20
   - `lidar_type`: try 1 (TYPE_TRIANGLE)
   - `support_motor_dtr`: try true

After changing config, rebuild:
```bash
cd ~/yahboomcar_ros2_ws/software/library_ws
colcon build --packages-select ydlidar_ros2_driver
```
