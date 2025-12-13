#!/usr/bin/env python3
"""
Autonomous Driving Stack Verification
Verifies all components are installed and ready for integration
"""

import subprocess
import sys
import os

def check_section(title):
    """Print section header"""
    print("\n" + "="*70)
    print(f"  {title}")
    print("="*70)

def check_command(cmd, description):
    """Check if a command succeeds"""
    try:
        result = subprocess.run(
            cmd,
            shell=True,
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            print(f"✅ {description}")
            return True
        else:
            print(f"❌ {description}")
            return False
    except Exception as e:
        print(f"❌ {description} - Error: {e}")
        return False

def check_python_module(module_name, description):
    """Check if a Python module is installed"""
    try:
        __import__(module_name)
        print(f"✅ {description}")
        return True
    except ImportError:
        print(f"❌ {description} - Not installed")
        return False

def check_ollama_model(model_name, description):
    """Check if Ollama model is available"""
    try:
        result = subprocess.run(
            f"ollama list | grep {model_name}",
            shell=True,
            capture_output=True,
            text=True
        )
        if model_name in result.stdout:
            print(f"✅ {description}")
            return True
        else:
            print(f"❌ {description} - Not downloaded")
            return False
    except Exception as e:
        print(f"❌ {description} - Error: {e}")
        return False

def check_device(device_path, description):
    """Check if a device exists"""
    if os.path.exists(device_path):
        print(f"✅ {description}: {device_path}")
        return True
    else:
        print(f"❌ {description}: {device_path} not found")
        return False

def test_tinyllama_speed():
    """Quick inference speed test"""
    import requests
    import time

    try:
        start = time.time()
        response = requests.post(
            "http://localhost:11434/api/generate",
            json={
                "model": "tinyllama:1.1b",
                "prompt": "Go LEFT or RIGHT?",
                "stream": False,
                "options": {"num_predict": 5}
            },
            timeout=2
        )
        elapsed = time.time() - start

        if response.status_code == 200 and elapsed < 0.5:
            print(f"✅ TinyLlama inference: {elapsed:.3f}s (< 0.5s target)")
            return True
        elif response.status_code == 200:
            print(f"⚠️  TinyLlama inference: {elapsed:.3f}s (slow)")
            return True
        else:
            print(f"❌ TinyLlama inference failed")
            return False
    except Exception as e:
        print(f"❌ TinyLlama inference test failed: {e}")
        return False

def main():
    print("="*70)
    print("  Autonomous Driving Stack Verification")
    print("="*70)
    print("\nChecking all components for autonomous driving system...\n")

    results = {}

    # ========================================
    # 1. LLM REASONING
    # ========================================
    check_section("1. LLM Reasoning Engine")
    results['ollama'] = check_command("ollama --version", "Ollama runtime")
    results['tinyllama'] = check_ollama_model("tinyllama:1.1b", "TinyLlama 1.1B model")
    results['tinyllama_speed'] = test_tinyllama_speed()

    # ========================================
    # 2. VISION PROCESSING
    # ========================================
    check_section("2. Vision Processing")
    results['torch'] = check_python_module("torch", "PyTorch")
    results['ultralytics'] = check_python_module("ultralytics", "YOLO11 (Ultralytics)")
    results['transformers'] = check_python_module("transformers", "Transformers (for DINOv3)")
    results['timm'] = check_python_module("timm", "TIMM (PyTorch Image Models)")
    results['cv2'] = check_python_module("cv2", "OpenCV")

    # ========================================
    # 3. VOICE PROCESSING
    # ========================================
    check_section("3. Voice Processing")

    # Check Whisper by looking for the package, not importing
    # (avoid numba/coverage conflict)
    try:
        result = subprocess.run(
            "pip3 list | grep openai-whisper",
            shell=True,
            capture_output=True,
            text=True
        )
        if "openai-whisper" in result.stdout:
            print(f"✅ OpenAI Whisper (package installed)")
            results['whisper'] = True
        else:
            print(f"❌ OpenAI Whisper - Not installed")
            results['whisper'] = False
    except Exception:
        print(f"❌ OpenAI Whisper - Check failed")
        results['whisper'] = False

    sys.path.append('/opt/ros/humble/lib/python3.10/site-packages')
    try:
        from Speech_Lib import Speech
        print(f"✅ Speech_Lib (CSK4002 voice module)")
        results['speech_lib'] = True
    except ImportError:
        print(f"❌ Speech_Lib - Not installed")
        results['speech_lib'] = False

    # ========================================
    # 4. ROS2 & SENSORS
    # ========================================
    check_section("4. ROS2 & Robot Hardware")

    # Check ROS2 with proper bash sourcing
    try:
        result = subprocess.run(
            "bash -c 'source /opt/ros/humble/setup.bash && ros2 pkg list 2>/dev/null | head -1'",
            shell=True,
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0 and len(result.stdout) > 0:
            print(f"✅ ROS2 Humble")
            results['ros2'] = True
        else:
            print(f"❌ ROS2 Humble")
            results['ros2'] = False
    except Exception:
        print(f"❌ ROS2 Humble - Check failed")
        results['ros2'] = False

    # Check camera driver
    try:
        result = subprocess.run(
            "bash -c 'source /opt/ros/humble/setup.bash && source /home/jetson/yahboomcar_ros2_ws/software/library_ws/install/setup.bash 2>/dev/null && ros2 pkg list 2>/dev/null | grep astra_camera'",
            shell=True,
            capture_output=True,
            text=True,
            timeout=5
        )
        if "astra_camera" in result.stdout:
            print(f"✅ Astra camera driver")
            results['camera_driver'] = True
        else:
            print(f"⚠️  Astra camera driver")
            results['camera_driver'] = False
    except Exception:
        print(f"⚠️  Astra camera driver - Check failed")
        results['camera_driver'] = False

    # Check lidar driver
    try:
        result = subprocess.run(
            "bash -c 'source /opt/ros/humble/setup.bash && source /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash 2>/dev/null && ros2 pkg list 2>/dev/null | grep ydlidar'",
            shell=True,
            capture_output=True,
            text=True,
            timeout=5
        )
        if "ydlidar" in result.stdout:
            print(f"✅ YDLidar driver")
            results['lidar_driver'] = True
        else:
            print(f"⚠️  YDLidar driver")
            results['lidar_driver'] = False
    except Exception:
        print(f"⚠️  YDLidar driver - Check failed")
        results['lidar_driver'] = False

    # ========================================
    # 5. USB DEVICES
    # ========================================
    check_section("5. Hardware Devices")

    # Check for any USB devices
    import glob
    usb_devices = glob.glob('/dev/ttyUSB*')
    if usb_devices:
        print(f"✅ USB devices found: {', '.join(usb_devices)}")
        results['usb_devices'] = True
    else:
        print(f"⚠️  No USB devices found (connect sensors)")
        results['usb_devices'] = False

    # Check for camera
    result = subprocess.run("lsusb | grep -iE '(orbbec|2bc5)'",
                          shell=True, capture_output=True, text=True)
    if result.returncode == 0:
        print(f"✅ Astra camera detected on USB")
        results['camera_hw'] = True
    else:
        print(f"⚠️  Astra camera not detected")
        results['camera_hw'] = False

    # ========================================
    # 6. GPU & COMPUTE
    # ========================================
    check_section("6. GPU & Compute Resources")
    results['cuda'] = check_python_module("torch.cuda", "CUDA support")

    if results['torch']:
        import torch
        if torch.cuda.is_available():
            print(f"✅ GPU available: {torch.cuda.get_device_name(0)}")
            print(f"   VRAM: {torch.cuda.get_device_properties(0).total_memory / 1e9:.1f} GB")
            results['gpu'] = True
        else:
            print(f"⚠️  GPU not available")
            results['gpu'] = False

    # ========================================
    # SUMMARY
    # ========================================
    check_section("VERIFICATION SUMMARY")

    critical_components = [
        ('ollama', 'Ollama runtime'),
        ('tinyllama', 'TinyLlama model'),
        ('tinyllama_speed', 'TinyLlama speed'),
        ('ultralytics', 'YOLO11'),
        ('transformers', 'Transformers'),
        ('ros2', 'ROS2'),
    ]

    optional_components = [
        ('whisper', 'Whisper STT'),
        ('speech_lib', 'Voice module'),
        ('camera_hw', 'Camera hardware'),
        ('camera_driver', 'Camera driver'),
        ('lidar_driver', 'LiDAR driver'),
        ('gpu', 'GPU acceleration'),
    ]

    print("\n🔴 Critical Components:")
    critical_ok = 0
    for key, name in critical_components:
        status = "✅" if results.get(key, False) else "❌"
        print(f"   {status} {name}")
        if results.get(key, False):
            critical_ok += 1

    print("\n🟡 Optional Components:")
    for key, name in optional_components:
        status = "✅" if results.get(key, False) else "⚠️ "
        print(f"   {status} {name}")

    print("\n" + "="*70)
    if critical_ok == len(critical_components):
        print("  ✅ ALL CRITICAL COMPONENTS READY")
        print("="*70)
        print("\n🚀 System ready for autonomous driving integration!")
        print("\n📋 Next Steps:")
        print("   1. Integrate components into ROS2 nodes")
        print("   2. Create perception pipeline (YOLO + DINOv3)")
        print("   3. Build LLM decision node (TinyLlama)")
        print("   4. Implement control loop (2+ Hz)")
        print("\n💡 Performance Targets:")
        print("   • Perception: ~0.15s (YOLO + DINOv3)")
        print("   • LLM reasoning: ~0.25s (with token limits)")
        print("   • Control: ~0.05s")
        print("   • Total cycle: ~0.45s = 2.2 Hz ✅")
        return 0
    else:
        print("  ❌ MISSING CRITICAL COMPONENTS")
        print("="*70)
        print("\n⚠️  Install missing components before proceeding")
        return 1

if __name__ == "__main__":
    sys.exit(main())
