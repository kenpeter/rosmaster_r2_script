#!/bin/bash

# Yahboom Robot USB Device Finder
# This script helps identify which USB cable is which

clear
echo "================================================================"
echo "  🔍 Yahboom Robot USB Device Finder"
echo "================================================================"
echo ""
echo "This script will help you identify which USB cable connects to"
echo "which device on your robot."
echo ""
echo "================================================================"
echo ""

# Function to check current USB devices
check_current_devices() {
    echo "📊 Currently connected USB devices:"
    echo "-----------------------------------"
    lsusb | grep -v "Hub\|Billboard" | nl -w2 -s". "
    echo ""
}

# Function to monitor for new USB devices
monitor_usb() {
    echo "🔌 Monitoring for USB device changes..."
    echo "   (Press Ctrl+C when done)"
    echo ""

    # Capture current state
    BEFORE=$(lsusb | sort)

    while true; do
        sleep 0.5
        AFTER=$(lsusb | sort)

        # Check for new devices
        NEW=$(comm -13 <(echo "$BEFORE") <(echo "$AFTER"))
        if [ -n "$NEW" ]; then
            echo ""
            echo "🆕 NEW DEVICE DETECTED!"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "$NEW"

            # Parse device info
            VENDOR_ID=$(echo "$NEW" | grep -oP 'ID \K[0-9a-f]{4}')
            PRODUCT_ID=$(echo "$NEW" | grep -oP 'ID [0-9a-f]{4}:\K[0-9a-f]{4}')
            DEVICE_NAME=$(echo "$NEW" | sed 's/^.*ID [0-9a-f]*:[0-9a-f]* //')

            echo ""
            echo "📋 Device Details:"
            echo "   Vendor ID:  $VENDOR_ID"
            echo "   Product ID: $PRODUCT_ID"
            echo "   Name:       $DEVICE_NAME"
            echo ""

            # Identify the device
            if [ "$VENDOR_ID" = "1a86" ] && ( [ "$PRODUCT_ID" = "7522" ] || [ "$PRODUCT_ID" = "7523" ] ); then
                echo "🤖 THIS IS YOUR ROBOT CONTROLLER!"
                echo "   ✅ CH340 USB-to-Serial chip detected"
                echo "   ✅ This cable controls motors, IMU, and encoders"
                echo "   ✅ Should create /dev/myserial"
            elif [ "$VENDOR_ID" = "10c4" ] && [ "$PRODUCT_ID" = "ea60" ]; then
                echo "🌀 THIS IS YOUR RPLIDAR!"
                echo "   ✅ CP2102 USB-to-Serial chip detected"
                echo "   ✅ This cable connects to the rotating laser scanner"
                echo "   ✅ Should create /dev/rplidar"
            elif [[ "$DEVICE_NAME" =~ "Orbbec" ]]; then
                echo "📷 THIS IS YOUR CAMERA!"
                echo "   ✅ Astra depth camera detected"
                echo "   ✅ Provides RGB and depth images"
            else
                echo "❓ Unknown device - might not be robot hardware"
            fi

            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo ""

            # Check if ttyUSB devices were created
            sleep 1
            NEW_TTY=$(ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | tail -1)
            if [ -n "$NEW_TTY" ]; then
                echo "💾 Serial device created:"
                echo "$NEW_TTY"
                echo ""
            fi

            # Check symlinks
            if [ -e /dev/myserial ]; then
                echo "🔗 /dev/myserial → $(readlink /dev/myserial)"
            fi
            if [ -e /dev/rplidar ]; then
                echo "🔗 /dev/rplidar → $(readlink /dev/rplidar)"
            fi
            echo ""

            BEFORE="$AFTER"
        fi

        # Check for removed devices
        REMOVED=$(comm -23 <(echo "$BEFORE") <(echo "$AFTER"))
        if [ -n "$REMOVED" ]; then
            echo ""
            echo "🔌 DEVICE DISCONNECTED!"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "$REMOVED"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo ""
            BEFORE="$AFTER"
        fi
    done
}

# Main menu
echo "What would you like to do?"
echo ""
echo "1) Show currently connected devices"
echo "2) Monitor for new devices (plug/unplug to identify)"
echo "3) Run full hardware check"
echo "4) Show current status"
echo ""
read -p "Enter choice [1-4]: " choice
echo ""

case $choice in
    1)
        check_current_devices
        echo ""
        echo "Legend:"
        echo "  1a86:7522/7523 = Robot Controller (CH340) → /dev/myserial"
        echo "  10c4:ea60      = RPLiDAR (CP2102)         → /dev/rplidar"
        echo "  Orbbec         = Astra Camera"
        ;;
    2)
        echo "================================================================"
        echo "📝 Instructions:"
        echo "================================================================"
        echo ""
        echo "1. The script will now monitor USB connections"
        echo "2. UNPLUG a USB cable from your robot"
        echo "3. Wait 2 seconds"
        echo "4. PLUG IT BACK IN"
        echo "5. The script will identify what it is"
        echo "6. Repeat for each cable"
        echo ""
        read -p "Press Enter to start monitoring..."
        echo ""
        monitor_usb
        ;;
    3)
        if [ -f "./check_hardware.sh" ]; then
            ./check_hardware.sh
        else
            echo "❌ check_hardware.sh not found in current directory"
        fi
        ;;
    4)
        echo "================================================================"
        echo "  Current Hardware Status"
        echo "================================================================"
        echo ""

        # Robot Controller
        if lsusb | grep -q "1a86:752[23]"; then
            echo "🤖 Robot Controller: ✅ CONNECTED"
            if [ -e /dev/myserial ]; then
                echo "   → /dev/myserial: ✅ $(readlink /dev/myserial)"
            else
                echo "   → /dev/myserial: ❌ NOT CREATED (check udev rules)"
            fi
        else
            echo "🤖 Robot Controller: ❌ NOT CONNECTED"
            echo "   → This is REQUIRED for motor control"
        fi
        echo ""

        # RPLiDAR
        if lsusb | grep -q "10c4:ea60"; then
            echo "🌀 RPLiDAR: ✅ CONNECTED"
            if [ -e /dev/rplidar ]; then
                echo "   → /dev/rplidar: ✅ $(readlink /dev/rplidar)"
            else
                echo "   → /dev/rplidar: ❌ NOT CREATED (check udev rules)"
            fi
        else
            echo "🌀 RPLiDAR: ❌ NOT CONNECTED"
        fi
        echo ""

        # Camera
        if lsusb | grep -q "Orbbec"; then
            echo "📷 Camera: ✅ CONNECTED"
        else
            echo "📷 Camera: ❌ NOT CONNECTED"
        fi
        echo ""

        echo "================================================================"
        echo ""
        echo "All ttyUSB devices:"
        ls -la /dev/ttyUSB* 2>/dev/null || echo "   (none found)"
        ;;
    *)
        echo "Invalid choice"
        exit 1
        ;;
esac

echo ""
echo "Done!"
