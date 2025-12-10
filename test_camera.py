#!/usr/bin/env python3
"""
Minimal Camera Test Script
Tests the Astra depth camera using OpenCV and ROS2 topics
"""

import subprocess
import sys
import time
import os

def test_usb_camera():
    """Test if camera is detected via USB"""
    print("\n[TEST 1] Checking USB devices...")
    result = subprocess.run(
        ["lsusb"],
        capture_output=True,
        text=True
    )

    # Look for Orbbec/Astra camera
    camera_found = False
    for line in result.stdout.split('\n'):
        if 'Orbbec' in line or '2bc5' in line:
            print(f"✅ Found Astra camera: {line.strip()}")
            camera_found = True
            break

    if not camera_found:
        print("⚠️  Astra camera not found in USB devices")
        print("   Looking for other video devices...")
        # Check for generic video devices
        result2 = subprocess.run(
            ["ls", "-la", "/dev/video*"],
            capture_output=True,
            text=True
        )
        if result2.returncode == 0:
            print(f"   Found video devices:\n{result2.stdout}")
        else:
            print("   No /dev/video* devices found")

    return camera_found

def test_ros2_camera_topics():
    """Test if camera topics are available in ROS2"""
    print("\n[TEST 2] Checking ROS2 camera topics...")

    # Check if any ROS2 nodes are running
    result = subprocess.run(
        ["ros2", "node", "list"],
        capture_output=True,
        text=True,
        timeout=3
    )

    if not result.stdout.strip():
        print("⚠️  No ROS2 nodes running - cannot check topics")
        print("   Start the robot first: ./scripts/rosmaster_r2_script/start_robot.sh")
        return False

    # Check for camera topics
    camera_topics = [
        '/camera/color/image_raw',
        '/camera/depth/image_raw',
        '/camera/color/camera_info',
        '/camera/depth/camera_info'
    ]

    topic_found = False
    for topic in camera_topics:
        result = subprocess.run(
            ["ros2", "topic", "info", topic],
            capture_output=True,
            text=True,
            timeout=2
        )
        if result.returncode == 0:
            print(f"✅ Topic active: {topic}")
            topic_found = True
        else:
            print(f"❌ Topic not found: {topic}")

    return topic_found

def test_opencv_capture():
    """Test camera using OpenCV"""
    print("\n[TEST 3] Testing camera with OpenCV...")

    try:
        import cv2
        print("✅ OpenCV imported successfully")

        # Try to open camera (usually /dev/video0 or video1)
        for video_id in [0, 1, 2]:
            print(f"\nTrying to open /dev/video{video_id}...")
            cap = cv2.VideoCapture(video_id)

            if cap.isOpened():
                print(f"✅ Camera opened on /dev/video{video_id}")

                # Try to read a frame
                ret, frame = cap.read()
                if ret:
                    print(f"✅ Successfully captured frame: {frame.shape}")
                    print("\n📸 Displaying camera preview for 5 seconds...")
                    print("   Press 'q' to quit early")

                    start_time = time.time()
                    while (time.time() - start_time) < 5:
                        ret, frame = cap.read()
                        if ret:
                            cv2.imshow('Camera Test', frame)
                            if cv2.waitKey(1) & 0xFF == ord('q'):
                                break

                    cv2.destroyAllWindows()
                    cap.release()
                    return True
                else:
                    print(f"❌ Could not read frame from /dev/video{video_id}")

                cap.release()
            else:
                print(f"❌ Could not open /dev/video{video_id}")

        return False

    except ImportError:
        print("❌ OpenCV (cv2) not installed")
        print("   Install with: pip3 install opencv-python")
        return False
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

def main():
    print("="*60)
    print("  CAMERA TEST - Astra Depth Camera")
    print("="*60)

    # Run tests
    usb_ok = test_usb_camera()

    # Check if running in GUI environment (needed for OpenCV display)
    if os.environ.get('DISPLAY'):
        opencv_ok = test_opencv_capture()
    else:
        print("\n[TEST 3] Skipped - No DISPLAY environment (SSH session?)")
        print("   Run this script locally on the Jetson for camera preview")
        opencv_ok = False

    ros2_ok = test_ros2_camera_topics()

    # Summary
    print("\n" + "="*60)
    print("  CAMERA TEST SUMMARY")
    print("="*60)
    print(f"\nUSB Detection:    {'✅ PASS' if usb_ok else '❌ FAIL'}")
    print(f"OpenCV Capture:   {'✅ PASS' if opencv_ok else '⚠️  SKIP/FAIL'}")
    print(f"ROS2 Topics:      {'✅ PASS' if ros2_ok else '❌ FAIL'}")

    if usb_ok or opencv_ok or ros2_ok:
        print("\n✅ Camera appears to be working!")
    else:
        print("\n❌ Camera issues detected:")
        print("   - Check USB connection")
        print("   - Check camera power")
        print("   - Try: lsusb | grep -i orbbec")
        print("   - Start ROS2 camera node if testing topics")

if __name__ == "__main__":
    main()
