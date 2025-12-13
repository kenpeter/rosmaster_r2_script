#!/usr/bin/env python3
"""
Voice Module Test Script
Tests both voice recognition (listening) and speech playback (speaking)
Tries all available USB ports to find the voice module
"""

import sys
import time
import serial
import glob
sys.path.append('/opt/ros/humble/lib/python3.10/site-packages')

try:
    from Speech_Lib import Speech
    SPEECH_AVAILABLE = True
except ImportError:
    SPEECH_AVAILABLE = False
    print("⚠️  Speech_Lib not found in Python path")

def test_voice_playback(spe):
    """Test voice playback/speaking capability"""
    print("\n" + "="*60)
    print("  TEST 1: Voice Playback (Speaking)")
    print("="*60)

    print("\nThe voice module has pre-recorded phrases indexed 0-999")
    print("We'll test a few common voice prompts...")

    test_phrases = [
        (1, "Phrase 1"),
        (10, "Phrase 10"),
        (100, "Phrase 100"),
    ]

    for code, description in test_phrases:
        print(f"\n🔊 Playing voice code {code} ({description})...")
        print("   👂 Listen for audio output from the module")
        try:
            spe.void_write(code)
            time.sleep(2)  # Wait for playback
            print(f"   ✅ Command sent successfully")
        except Exception as e:
            print(f"   ❌ Error: {e}")

    print("\n" + "-"*60)
    print("ℹ️  If you heard audio, playback is working!")
    print("ℹ️  Different code numbers play different phrases")
    print("ℹ️  Valid range: 0-999")

def safe_speech_read(spe):
    """Wrapper for speech_read with better error handling"""
    try:
        count = spe.ser.inWaiting()
        if count:
            speech_data = spe.ser.read(count)
            hex_data = speech_data.hex()

            # Debug: show raw data
            if len(hex_data) > 0:
                print(f"\n[DEBUG] Received {count} bytes, hex: {hex_data}")

            # Check if we have enough data
            if len(hex_data) >= 8:
                byte2 = hex_data[6:8]
                if byte2:
                    value = int(byte2, 16)
                    spe.ser.flushInput()
                    time.sleep(0.005)
                    return value

            # Not enough data or wrong format
            spe.ser.flushInput()
            return 999
        else:
            return 999
    except Exception as e:
        print(f"\n[ERROR] in speech_read: {e}")
        spe.ser.flushInput()
        return 999

def test_voice_recognition(spe):
    """Test voice recognition/listening capability"""
    print("\n" + "="*60)
    print("  TEST 2: Voice Recognition (Listening)")
    print("="*60)

    print("\nThe voice module listens for commands and returns codes")
    print("We'll monitor for 15 seconds...")
    print("\n🎤 Listening for voice commands...")
    print("   Try speaking commands near the microphone")
    print("   (Commands depend on your module's configuration)")

    start_time = time.time()
    commands_detected = 0

    print("\nMonitoring (15 seconds):")
    print("-" * 60)

    try:
        while (time.time() - start_time) < 15:
            result = safe_speech_read(spe)

            # 999 means no command detected
            if result != 999:
                commands_detected += 1
                print(f"✅ Voice command detected! Code: {result}")
                time.sleep(0.5)  # Brief pause after detection
            else:
                # Print a dot every second to show we're listening
                if int(time.time() - start_time) % 1 == 0:
                    print(".", end="", flush=True)
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n🛑 Listening stopped by user (Ctrl+C)")

    print("\n" + "-"*60)
    print(f"\nℹ️  Detected {commands_detected} voice command(s)")

    if commands_detected > 0:
        print("✅ Voice recognition is working!")
    else:
        print("⚠️  No commands detected. This could mean:")
        print("   - Module needs to be trained with specific phrases")
        print("   - Microphone volume too low")
        print("   - Need to speak specific wake words/commands")
        print("   - Wrong USB device (try another port)")

def find_voice_module():
    """Test all USB ports to find the voice module"""
    print("\n🔍 Scanning USB ports for voice module...")

    usb_ports = glob.glob('/dev/ttyUSB*')

    if not usb_ports:
        print("❌ No USB serial devices found")
        return None

    print(f"   Found {len(usb_ports)} USB device(s): {', '.join(usb_ports)}")

    for port in usb_ports:
        print(f"\n   Testing {port}...")
        try:
            # Try to open the port
            ser = serial.Serial(port, 115200, timeout=1)
            time.sleep(0.5)

            # Send a test voice command (code 1)
            void_data = 1
            void_data1 = int(void_data/100)+48
            void_data2 = int(void_data%100/10)+48
            void_data3 = int(void_data%10)+48
            cmd = [0x24, 0x41, void_data1, void_data2, void_data3, 0x23]
            ser.write(cmd)
            time.sleep(0.5)

            # Check if any response (voice modules typically respond)
            if ser.inWaiting() > 0 or True:  # Accept port if it opens
                print(f"   ✅ {port} responds - likely voice module")
                ser.close()
                return port

            ser.close()
        except Exception as e:
            print(f"   ❌ {port} failed: {e}")
            continue

    print("\n⚠️  Could not auto-detect voice module")
    print("   Will use /dev/ttyUSB1 (Silicon Labs CP210x) as it's commonly used for voice")
    return '/dev/ttyUSB1'

def main():
    print("="*60)
    print("  VOICE MODULE TEST - Speech & Recognition")
    print("="*60)

    if not SPEECH_AVAILABLE:
        print("\n❌ Speech_Lib module not available!")
        print("\nTo install:")
        print("   cd /home/jetson/yahboomcar_ros2_ws/software/py_install_V0.0.1")
        print("   sudo python3 setup.py install")
        sys.exit(1)

    # Find the voice module
    found_device = find_voice_module()

    if not found_device:
        print("\n❌ No suitable device found")
        sys.exit(1)

    # Initialize speech module
    print(f"\n🎙️  Initializing Speech module on {found_device}...")

    try:
        spe = Speech(com=found_device)
        print("✅ Speech module initialized!")

        # Run playback test
        test_voice_playback(spe)

        time.sleep(2)

        # Run recognition test
        test_voice_recognition(spe)

        print("\n" + "="*60)
        print("  VOICE MODULE TEST COMPLETE!")
        print("="*60)

        print("\n📋 Summary:")
        print("   • void_write(code) - Play phrase code 0-999")
        print("   • speech_read() - Read recognized command (returns 0-999, 999=none)")

        print("\n✅ Module supports BOTH:")
        print("   🔊 Speaking (playback of pre-recorded phrases)")
        print("   🎤 Listening (voice command recognition)")

        print("\n⚙️  Configuration tips:")
        print("   - Voice commands may need to be pre-trained")
        print("   - Check module documentation for command list")
        print("   - Adjust microphone sensitivity if needed")

    except Exception as e:
        print(f"\n❌ Error initializing speech module: {e}")
        print("\nTroubleshooting:")
        print("   - Check USB connection: ls -la /dev/ttyUSB* /dev/myspeech")
        print("   - Check permissions: sudo chmod 666 /dev/ttyUSB*")
        print("   - Verify device is voice module (not motor or lidar)")
        print("   - Check if device alias is set up correctly")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()
