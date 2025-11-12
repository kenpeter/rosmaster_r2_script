# ROS Master R2 Troubleshooting Guide

## Problem Summary

**Issue 1: Motors Not Working**
- ROS publishes `/cmd_vel` messages successfully ✅
- Driver node receives messages ✅
- BUT motors don't move ❌

**Issue 2: Phone App Not Connecting**
- IP address 192.168.1.11 was previously working
- Now app can't connect

## Root Cause

**The ROS Master board is not communicating properly with the Jetson.**

### Evidence
```
Firmware version: -1 (should be a positive number like 3.3)
Battery voltage: 0.0V (should show actual voltage like 12.5V)
```

This means:
- Serial port opens successfully ✅
- But the board doesn't respond to commands ❌
- Board firmware may have crashed or needs reset

## Hardware Setup

Your system has:
- **ttyUSB0**: YDLidar (Silicon Labs CP2102)
- **ttyUSB1**: ROS Master board (QinHeng CH340)
- **/dev/myserial**: Symlink → ttyUSB1

## Diagnostic Scripts Created

### 1. Find ROS Master Port
Tests both USB ports to find which has the ROS Master board:
```bash
./find_rosmaster_port.py
```

### 2. Reset ROS Master Board
Attempts to reset and recover board communication:
```bash
./reset_rosmaster.py
```

### 3. Test Motors Directly
Tests motor control using Rosmaster library directly (bypassing ROS):
```bash
./test_motors_direct.py
```

### 4. Advanced Serial Test
Low-level serial communication test:
```bash
./test_rosmaster_advanced.py
```

## Troubleshooting Steps

### Step 1: Identify the Correct Port
```bash
cd ~/yahboomcar_ros2_ws/yahboomcar_ws/scripts
./find_rosmaster_port.py
```

This will test both ttyUSB0 and ttyUSB1 to find which one responds.

**If it finds the ROS Master on a different port**, update the symlink:
```bash
sudo ln -sf ttyUSB0 /dev/myserial  # If found on ttyUSB0
# or
sudo ln -sf ttyUSB1 /dev/myserial  # If found on ttyUSB1
```

### Step 2: Power Cycle the Board

**IMPORTANT**: The ROS Master board may need a power cycle.

**Option A - Full Power Cycle (RECOMMENDED):**
1. Disconnect the ROS Master board power cable (not just USB)
2. Wait 10 seconds
3. Reconnect the power cable
4. Wait for the board to boot (~5 seconds)
5. Look for LED activity on the board

**Option B - USB Reset:**
1. Unplug USB cable from Jetson
2. Wait 5 seconds
3. Plug back in
4. Wait for device enumeration

### Step 3: Run Reset Script
```bash
./reset_rosmaster.py
```

This will:
- Send a reset command to the board
- Reconfigure car type
- Enable auto-report
- Test communication

### Step 4: Test Direct Motor Control
```bash
./test_motors_direct.py
```

This tests if motors work when controlled directly (without ROS).

**Expected results if working:**
- Firmware version: 3.3 (or similar)
- Battery voltage: 8V-12V (actual battery voltage)
- Hear beep sound
- Wheels move forward and rotate

### Step 5: Restart ROS System
If direct motor control works:
```bash
./start_robot.sh
```

Then test ROS control:
```bash
./test_motor_continuous.sh
```

## Common Issues and Solutions

### Issue: Version = -1, Voltage = 0V

**Cause**: Board not responding to commands

**Solutions**:
1. ✅ **Power cycle the ROS Master board** (most common fix)
2. Check USB cable connection
3. Try different USB port on Jetson
4. Check board power LEDs
5. Run `./reset_rosmaster.py`

### Issue: Board responds but motors don't move

**Cause**: Motor power issue (not communication)

**Check**:
1. **Battery connected and charged?** (needs >7V)
2. **Motor power LED lit?** (on motor driver section)
3. **Motor connectors plugged in?** (check 4 motor cables)
4. **Emergency stop active?** (if equipped)

### Issue: Wrong USB port

**Symptoms**: Opening serial works but no response

**Solution**: Run `./find_rosmaster_port.py` to identify correct port

### Issue: Phone app can't connect

**Cause**: The phone app connects to a separate TCP server (`test.py`), not the ROS system

**Note**: The message "TCP Service IP= 192.168.1.11" you saw is from a standalone Python script that runs a TCP server for the phone app. This is separate from the ROS driver.

**The phone app issue is a symptom of the same root cause**: The ROS Master board communication is broken, so the app can't get status/control.

**Solution**: Fix the board communication first (steps above), then the app should work.

## Hardware Checks

### Visual Inspection
1. **ROS Master Board Power LED**: Should be lit (usually green)
2. **Motor Driver LEDs**: Should be lit when powered
3. **USB Connection**: Firmly seated in both ends
4. **Battery**: Charged and connected

### Electrical Checks
1. **Battery Voltage**: Use multimeter, should be 7V-12V
2. **Board Power**: Check 5V rail on board
3. **Motor Connections**: All 4 motors connected

## When to Contact Support

Contact Yahboom support if:
1. Board power LED is off (hardware failure)
2. Power cycling doesn't help
3. No response on any USB port
4. Visible damage to board
5. Firmware appears corrupted

## Advanced: Firmware Reflashing

If nothing else works, the firmware may need to be reflashed. Contact Yahboom for:
- Firmware binary file
- Flashing instructions for your board model
- Serial bootloader procedure

## Summary of Created Tools

| Script | Purpose |
|--------|---------|
| `find_rosmaster_port.py` | Find which USB port has ROS Master |
| `reset_rosmaster.py` | Reset and recover board communication |
| `test_motors_direct.py` | Test motors using library directly |
| `test_rosmaster_advanced.py` | Low-level serial diagnostics |
| `test_rosmaster_serial.py` | Basic serial port test |

## Quick Reference Commands

```bash
# Stop robot
./stop_robot.sh

# Find ROS Master port
./find_rosmaster_port.py

# Reset board
./reset_rosmaster.py

# Test motors directly
./test_motors_direct.py

# Start robot
./start_robot.sh

# Test ROS motor control
./test_motor_continuous.sh

# Check ROS topics
ros2 topic echo /edition --once   # Should show version number
ros2 topic echo /voltage --once   # Should show battery voltage
ros2 node info /driver_node       # Show driver status
```

## Next Steps

Based on your test results (Version: -1, Voltage: 0V, no beep, no movement):

**Priority 1**: Power cycle the ROS Master board
**Priority 2**: Run `./find_rosmaster_port.py`
**Priority 3**: Run `./reset_rosmaster.py`
**Priority 4**: Test with `./test_motors_direct.py`

If all diagnostics fail, the board likely needs:
- Firmware reflashing
- Hardware repair
- Replacement

---

*Created: 2025-11-12*
*For: Yahboom ROS Master R2 Robot*
