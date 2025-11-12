#!/bin/bash
# =============================================================================
# check_rosmaster_board.sh
# Comprehensive health-check for Yahboom ROSMASTER R2 controller board
# Works on Jetson (Ubuntu 20.04/22.04 + ROS2 Humble/Foxy)
# =============================================================================
# Author:  Grok (2025)
# License: MIT
# =============================================================================

set -euo pipefail

# -------------------------- CONFIG --------------------------
ROSMASTER_VID="0483"          # STM32 Virtual COM port (ST Microelectronics)
ROSMASTER_PID="5740"          # Common PID for ROSMASTER R2
EXPECTED_TTY="/dev/rosmaster" # Desired symlink (will be created if missing)
BAUD=115200
TIMEOUT=3
PYTHON_SCRIPT=$(cat <<'PY'
import serial, sys, time
def send_cmd(ser, cmd, expect=None, timeout=2):
    ser.write((cmd + '\n').encode())
    time.sleep(0.1)
    resp = ''
    t0 = time.time()
    while time.time() - t0 < timeout:
        if ser.in_waiting:
            resp += ser.read(ser.in_waiting).decode(errors='ignore')
        if expect and expect in resp:
            return resp.strip()
        time.sleep(0.05)
    return resp.strip()
def main(port):
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(2)  # Arduino-style reset delay
        print("   Connected to board")
        # 1. Firmware handshake
        ver = send_cmd(ser, "VER?", "ROSMASTER")
        print(f"   Firmware: {ver or 'NO RESPONSE'}")
        # 2. Motor driver test
        send_cmd(ser, "MOTOR TEST", "OK")
        print("   Motor self-test: PASSED" if "OK" in ver else "   Motor self-test: FAILED")
        # 3. Encoder read
        enc = send_cmd(ser, "ENC?", timeout=1)
        print(f"   Encoders: {enc or 'NO DATA'}")
        # 4. Power rail check
        volt = send_cmd(ser, "VOLT?", timeout=1)
        print(f"   5V rail: {volt or 'NO DATA'}")
        ser.close()
        return 0
    except Exception as e:
        print(f"   ERROR: {e}")
        return 1
if __name__ == '__main__':
    sys.exit(main(sys.argv[1]))
PY
)
# -----------------------------------------------------------

log() { echo -e "\033[1;34m==>\033[0m $*"; }
ok()  { echo -e "   \033[1;32mPASS\033[0m $*"; }
warn(){ echo -e "   \033[1;33mWARN\033[0m $*"; }
err() { echo -e "   \033[1;31mFAIL\033[0m $*"; }

log "Yahboom ROSMASTER R2 Board Diagnostic"
echo "---------------------------------------------------"

# 1. USB enumeration
log "1. Checking USB enumeration..."
if lsusb | grep -q "${ROSMASTER_VID}:${ROSMASTER_PID}"; then
    ok "USB device found (VID:PID = $ROSMASTER_VID:$ROSMASTER_PID)"
else
    err "USB device NOT detected!"
    echo "   -> Check cable, try another USB port, or power cycle the board."
    echo "   -> Run: lsusb"
    exit 1
fi

# 2. Find serial port
log "2. Locating serial port..."
PORT=$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | grep -m1 -E 'ACM|USB' || true)
if [ -z "$PORT" ]; then
    err "No ttyACM/USB device found!"
    exit 1
fi
ok "Serial port: $PORT"

# 3. udev symlink
log "3. Creating/validating udev symlink..."
if [ ! -e "$EXPECTED_TTY" ]; then
    sudo ln -sf "$PORT" "$EXPECTED_TTY" && ok "Symlink $EXPECTED_TTY -> $PORT"
else
    if [ "$(readlink -f $EXPECTED_TTY)" = "$PORT" ]; then
        ok "Symlink $EXPECTED_TTY is correct"
    else
        warn "Symlink points to wrong device, fixing..."
        sudo ln -sf "$PORT" "$EXPECTED_TTY" && ok "Fixed"
    fi
fi

# 4. Permissions
log "4. Checking user permissions..."
if [ -r "$PORT" ] && [ -w "$PORT" ]; then
    ok "User has r/w access"
else
    warn "Adding user to dialout group..."
    sudo usermod -aG dialout $USER
    echo "   -> Log out/in or run: newgrp dialout"
fi

# 5. Python diagnostic
log "5. Running firmware-level tests (Python)..."
TMP_PY=$(mktemp /tmp/check_rosmaster_XXXX.py)
echo "$PYTHON_SCRIPT" > "$TMP_PY"
if python3 "$TMP_PY" "$PORT"; then
    ok "Board firmware responded correctly"
else
    err "Board firmware test FAILED"
    echo "   -> Possible causes:"
    echo "        - Wrong firmware (not ROSMASTER R2)"
    echo "        - Damaged MCU / UART pins"
    echo "        - Power issue (check 5V rail with multimeter)"
fi
rm -f "$TMP_PY"

# 6. Motor direct test (optional, non-destructive)
log "6. Optional motor wiggle test (5s)..."
read -t 5 -p "   Press ENTER within 5s to SKIP, or wait to run..." || true
if [ $? -eq 0 ]; then
    ok "Skipped"
else
    echo "   Sending MOTOR TEST command..."
    echo -e "MOTOR TEST\n" > "$PORT" 2>/dev/null || true
    sleep 2
    ok "Sent (listen for motor buzz)"
fi

# -------------------------- SUMMARY --------------------------
echo "---------------------------------------------------"
log "DIAGNOSTIC SUMMARY"
if lsusb | grep -q "${ROSMASTER_VID}:${ROSMASTER_PID}" && \
   [ -e "$EXPECTED_TTY" ] && \
   python3 "$TMP_PY" "$PORT" >/dev/null 2>&1; then
    echo -e "\033[1;32mBOARD IS HEALTHY\033[0m"
else
    echo -e "\033[1;31mBOARD IS LIKELY BROKEN\033[0m"
    echo "   -> Try: new USB cable, different Jetson port, or RMA the board"
fi
echo "---------------------------------------------------"
