# USB Device Mapping - CRITICAL REFERENCE

**DO NOT CHANGE THIS MAPPING - IT IS CORRECT AND WORKING**

## CRITICAL RULES - READ BEFORE ANY CODE CHANGES

1. **YDLIDAR TG30 BAUDRATE = 512000**
   - NEVER use 230400 for YDLidar TG30
   - NEVER use any baudrate other than 512000
   - Check ALL test scripts, drivers, and config files
   - If you see 230400 in any ydlidar code, FIX IT to 512000

2. **Motor Controller BAUDRATE = 115200**
   - Robot motor uses 115200
   - Do not confuse with lidar baudrate

3. **USB Device Assignment (DO NOT SWAP)**
   - Motor → ttyUSB0 (CH340, 1a86:7523)
   - Lidar → ttyUSB1 (CP2102, 10c4:ea60)

## Device Assignment (VERIFIED WORKING)

```
/dev/myserial  → ttyUSB0 (CH340, ID: 1a86:7523) - Robot Motor Controller
/dev/ydlidar   → ttyUSB1 (CP2102, ID: 10c4:ea60) - YDLidar TG30
```

## Details

### Robot Motor Controller
- **Symlink:** `/dev/myserial`
- **Physical:** `ttyUSB0`
- **Chip:** CH340 (USB-to-Serial)
- **Vendor ID:** `1a86:7523`
- **Baudrate:** 115200
- **Used by:**
  - `Ackman_driver_R2` (motor control)
  - `test_motor.py` (hardware test)
  - Rosmaster_Lib library
- **Verified:** Motors respond to commands on this port

### YDLidar TG30
- **Symlink:** `/dev/ydlidar`
- **Physical:** `ttyUSB1`
- **Chip:** CP2102 (USB-to-Serial)
- **Vendor ID:** `10c4:ea60`
- **Baudrate:** 512000 (CRITICAL - do not use 230400)
- **Used by:**
  - `ydlidar_direct_node.py` (Lidar driver)
  - `test_ydlidar.py` (hardware test)
  - Launch file: `yahboomcar_bringup_R2_full_launch.py`
- **Verified:** Lidar publishes scan data on this port

## How to Verify

### Check current mapping:
```bash
ls -la /dev/myserial /dev/ydlidar
```

### Check USB device IDs:
```bash
udevadm info /dev/ttyUSB0 | grep -E "ID_VENDOR_ID|ID_MODEL_ID"
udevadm info /dev/ttyUSB1 | grep -E "ID_VENDOR_ID|ID_MODEL_ID"
```

### Expected output:
```
ttyUSB0: ID_VENDOR_ID=1a86, ID_MODEL_ID=7523  (CH340 - Robot)
ttyUSB1: ID_VENDOR_ID=10c4, ID_MODEL_ID=ea60  (CP2102 - Lidar)
```

## UDEV Rules (Automatic Symlink Creation)

### Rules Files Location:
- `/etc/udev/rules.d/yahboom_serial.rules` - Motor controller
- `/etc/udev/rules.d/ydlidar.rules` - YDLidar

### Motor Controller Rule (`/etc/udev/rules.d/yahboom_serial.rules`):
```bash
# Motor controller (CH340 chip - QinHeng Electronics)
# This is the correct chip - confirmed working on ttyUSB0
# Motor controller BAUDRATE = 115200
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", GROUP:="dialout", SYMLINK+="myserial"
```

### YDLidar Rule (`/etc/udev/rules.d/ydlidar.rules`):
```bash
# YDLIDAR TG30 (CP2102 USB Serial chip - Silicon Labs 10c4:ea60)
# Corrected from previous CH340 (1a86:7523) which is actually the motor controller
# CRITICAL: YDLidar TG30 BAUDRATE = 512000 (NEVER use 230400)
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"
```

### Reload udev rules (if modified):
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Fix Symlinks (if broken)

```bash
sudo rm /dev/myserial /dev/ydlidar
sudo ln -s ttyUSB0 /dev/myserial  # CH340 = Robot controller
sudo ln -s ttyUSB1 /dev/ydlidar   # CP2102 = YDLidar
```

## Testing

### Test motors:
```bash
python3 scripts/test_motor.py
```
Should move robot forward, backward, and rotate.

### Test Lidar:
```bash
./scripts/start_robot.sh
# In another terminal:
ros2 topic echo /scan --once
```
Should show laser scan data.

---

**Last verified:** 2025-12-29
**Status:** WORKING - DO NOT MODIFY
