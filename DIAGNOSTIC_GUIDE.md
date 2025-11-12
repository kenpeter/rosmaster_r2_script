# ROS Master Board Comprehensive Diagnostic Guide

## Quick Start

```bash
cd ~/yahboomcar_ros2_ws/yahboomcar_ws/scripts
sudo ./comprehensive_rosmaster_test.py
```

## What This Script Tests

### 1. Hardware Detection Tests
- ✓ USB device detection
- ✓ USB device permissions
- ✓ USB device information (vendor, model, serial)

### 2. Communication Tests
- ✓ ROS Master port identification
- ✓ Serial protocol validation
- ✓ Communication reliability (90%+ expected)
- ✓ Response time measurement (<50ms optimal)

### 3. Firmware Tests
- ✓ Firmware version check
- ✓ Command acknowledgment

### 4. Power System Tests
- ✓ Battery voltage reading
- ✓ Power status

### 5. Actuator Tests
- ✓ Motor control commands (safe, no actual movement)
- ✓ Servo control commands

### 6. Sensor Tests
- ✓ General sensor data retrieval
- ✓ IMU/orientation data

### 7. Stress Tests
- ✓ Continuous communication (5 seconds)
- ✓ Response time consistency

## Test Results

### Success Criteria
- **EXCELLENT (90-100%)**: Board fully functional, all systems operational
- **GOOD (70-89%)**: Board mostly functional, minor issues present
- **FAIR (50-69%)**: Board partially functional, significant issues
- **POOR (<50%)**: Board has serious problems, needs attention

### Output Format
- ✓ PASS (green): Test passed successfully
- ⚠ WARNING (yellow): Test passed with concerns
- ✗ FAIL (red): Test failed

## What Gets Tested

### Test 1: USB Device Detection
**What it does**: Scans for /dev/ttyUSB* devices
**What success means**: Your USB devices are connected and recognized by the system
**If it fails**:
- Check USB cable connections
- Try different USB ports
- Check if cables are damaged

### Test 2: USB Device Permissions
**What it does**: Checks if you have read/write access to USB devices
**What success means**: Your user has proper permissions
**If it fails**:
```bash
sudo usermod -a -G dialout $USER
# Then logout and login again
```

### Test 3: USB Device Information
**What it does**: Retrieves vendor ID, model, and serial number
**What success means**: Devices are properly enumerated
**Useful for**: Identifying which device is which

### Test 4: ROS Master Port Identification
**What it does**: Sends test commands to each USB port to find the ROS Master
**What success means**: ROS Master board is responding
**If it fails**:
- Power cycle the board
- Check if board has power (LEDs on?)
- Try different USB cable
- Check if firmware is corrupted

### Test 5: Serial Communication Protocol
**What it does**: Tests multiple command types (version, battery, etc.)
**What success means**: Board understands and responds to commands
**If it fails**:
- Firmware may be corrupted
- Wrong baud rate
- Communication protocol mismatch

### Test 6: Firmware Version Check
**What it does**: Retrieves firmware version from board
**What success means**: Board has valid firmware loaded
**Useful for**: Checking if firmware needs updating

### Test 7: Battery Voltage Reading
**What it does**: Reads battery voltage level
**What success means**: Power monitoring system working
**If it fails**:
- Battery not connected
- Voltage sensor issue

### Test 8: Motor Control Commands
**What it does**: Sends motor control commands (SAFE - no actual movement)
**What success means**: Motor controller is responsive
**Note**: Commands are sent but motors won't move (safety feature)

### Test 9: Servo Control Commands
**What it does**: Sends servo control commands
**What success means**: Servo controller is responsive

### Test 10: Sensor Data Retrieval
**What it does**: Requests general sensor data
**What success means**: Sensor system is operational

### Test 11: IMU Data Reading
**What it does**: Requests IMU/gyro/accelerometer data
**What success means**: IMU sensor is working
**Useful for**: Navigation and orientation

### Test 12: Continuous Communication Test
**What it does**: Sends 50+ commands over 5 seconds
**What success means**: Communication is stable and reliable
**Expected**: 90%+ success rate
**If poor**:
- USB cable quality issue
- Electrical interference
- USB hub problems (try direct connection)

### Test 13: Response Time Measurement
**What it does**: Measures how fast the board responds
**What success means**: Board is processing commands efficiently
**Excellent**: <50ms average
**Good**: 50-100ms average
**Slow**: >100ms (check USB cable, interference)

## Interpreting Results

### All Tests Pass
Your ROS Master board is working perfectly! You can proceed with normal operation.

### Hardware Tests Fail
**Problem**: Physical connection or device detection
**Solutions**:
1. Check all USB cables
2. Try different USB ports
3. Check device power
4. Verify cable quality

### Communication Tests Fail
**Problem**: Board not responding or responding incorrectly
**Solutions**:
1. Power cycle the board (unplug, wait 10 sec, replug)
2. Check firmware version
3. Try firmware reflash
4. Check for hardware damage

### Reliability <90%
**Problem**: Intermittent communication
**Solutions**:
1. Replace USB cable with high-quality one
2. Remove USB hubs, connect directly
3. Check for electrical interference
4. Move away from other electronic devices

### Slow Response Time
**Problem**: Commands taking too long
**Solutions**:
1. Check USB cable quality
2. Remove USB hubs
3. Close other programs using USB
4. Check system CPU load

## Output Files

The script generates a detailed report at:
```
/tmp/rosmaster_diagnostic_YYYYMMDD_HHMMSS.txt
```

This report includes:
- Test statistics
- Success/failure rates
- Detailed test results in JSON format
- Recommendations

## Common Issues & Solutions

### Issue: No USB Devices Found
**Symptoms**: Test 1 fails, no /dev/ttyUSB* devices
**Solutions**:
1. `lsusb` - Check if device appears at all
2. Check USB cable connection
3. Try different USB port
4. Check if device has power
5. `dmesg | tail` - Check for USB errors

### Issue: Permission Denied
**Symptoms**: Test 2 fails, can't access devices
**Solutions**:
```bash
sudo usermod -a -G dialout $USER
# Logout and login again, or reboot
```

### Issue: ROS Master Not Found
**Symptoms**: Test 4 fails, no response from any port
**Solutions**:
1. Power cycle the board:
   ```bash
   # Unplug USB, wait 10 seconds, replug
   ```
2. Check if board LEDs are on
3. Try different USB cable
4. Check board power supply
5. Firmware may need reflashing

### Issue: Communication Unreliable (<90%)
**Symptoms**: Test 12 shows low success rate
**Solutions**:
1. Replace USB cable with shielded, high-quality cable
2. Remove USB hubs, connect directly to computer
3. Check for nearby interference sources (WiFi routers, motors)
4. Try different USB port
5. Check USB cable length (shorter is better)

### Issue: Slow Response Time (>100ms)
**Symptoms**: Test 13 shows high response times
**Solutions**:
1. Close other programs
2. Check CPU usage: `htop`
3. Replace USB cable
4. Remove USB hubs
5. Check for system resource issues

### Issue: No Sensor Data
**Symptoms**: Tests 10-11 fail
**Solutions**:
1. Sensors may not be connected to board
2. Check sensor cable connections
3. Firmware may not support sensors
4. Sensor initialization may have failed

### Issue: Motor Commands Fail
**Symptoms**: Test 8 fails
**Solutions**:
1. Motor controller may be damaged
2. Firmware issue
3. Communication protocol mismatch
4. Try firmware reflash

## Advanced Diagnostics

### Check USB Connection Stability
```bash
# Monitor USB events in real-time
sudo dmesg -w
# In another terminal, wiggle the USB cable
# Look for disconnect/reconnect messages
```

### Check USB Device Details
```bash
lsusb -v | grep -A 20 "ttyUSB"
```

### Monitor Serial Communication
```bash
# Install if needed: sudo apt install minicom
minicom -D /dev/ttyUSB0 -b 115200
```

### Check for Conflicting Processes
```bash
# See what's using the serial port
sudo lsof | grep ttyUSB
```

## Integration with Other Scripts

This comprehensive test should be run:
- **Before**: Starting ROS operations
- **After**: Physical changes (cable replacement, board reconnection)
- **When**: Experiencing issues with robot operation
- **Periodically**: As part of maintenance routine

Complement with other diagnostic scripts:
```bash
./find_rosmaster_port.py        # Quick port identification
./test_motors_direct.py          # Actual motor movement test
./check_hardware.sh              # General hardware check
```

## What the Results Mean for ROS

### If diagnostics show EXCELLENT
- ROS nodes should connect properly
- `/cmd_vel` commands will work
- Sensor data will be reliable
- Ready for autonomous operation

### If diagnostics show GOOD
- ROS will mostly work
- May experience occasional issues
- Monitor for warnings
- Address warnings before autonomous operation

### If diagnostics show FAIR or POOR
- DO NOT use for autonomous operation
- Fix issues before running ROS
- Risk of unreliable behavior
- May cause unexpected robot movements

## Report Interpretation

The JSON report includes:
```json
{
  "hardware": {
    "usb_devices": [...],      // List of detected devices
    "permissions": "OK",        // Permission status
    "usb_info": "..."          // Device details
  },
  "communication": {
    "port": "/dev/ttyUSB0",    // Identified ROS Master port
    "protocol": "OK",           // Protocol status
    "reliability": 98.5,        // Success rate %
    "avg_response_time": 23.4   // Response time in ms
  },
  "firmware": {
    "version": "..."            // Firmware version hex
  },
  "power": {
    "battery_voltage": "..."    // Battery voltage hex
  },
  "actuators": {
    "motors": "TESTED",         // Motor status
    "servos": "TESTED"          // Servo status
  },
  "sensors": {
    "data": "...",              // Sensor data hex
    "imu": "..."                // IMU data hex
  }
}
```

## Next Steps After Diagnostics

### If All Tests Pass
1. Start ROS: `./scripts/start_robot.sh`
2. Test with teleop: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
3. Verify sensor data: `ros2 topic echo /imu`

### If Some Tests Fail
1. Follow recommendations in the report
2. Fix issues identified
3. Re-run diagnostics
4. Only proceed when GOOD or EXCELLENT

### If Most Tests Fail
1. Power cycle the entire system
2. Check all physical connections
3. Consider firmware reflash
4. Contact Yahboom support if hardware failure suspected

## Getting Help

If diagnostics show persistent failures:
1. Save the diagnostic report file
2. Check TROUBLESHOOTING.md
3. Review Yahboom documentation
4. Contact support with report file attached

## Safety Notes

- This script sends motor commands but they're designed NOT to cause movement
- Some actuator tests may cause tiny movements
- Ensure robot is on blocks or has room
- Stop command is sent before and after all motor tests
- Serial port is properly closed on exit

## Maintenance Schedule

Run this comprehensive diagnostic:
- **Weekly**: During active development
- **Monthly**: During normal operation
- **Before important demos/operations**
- **After any hardware changes**
- **When experiencing issues**

Keep diagnostic reports for:
- Tracking board health over time
- Identifying degrading performance
- Support requests
- Documentation

---

**Version**: 1.0
**Last Updated**: 2025-11-12
**Compatible with**: Yahboom ROS Master Board, ROS2
