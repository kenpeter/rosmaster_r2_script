#!/usr/bin/env python3
"""
Quick Power Diagnostic for Yahboom Rosmaster R2
Checks battery voltage and power connections
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import sys
import time

class PowerChecker(Node):
    def __init__(self):
        super().__init__('power_checker')
        self.voltage = None
        self.subscription = self.create_subscription(
            Float32,
            '/voltage',
            self.voltage_callback,
            10)

    def voltage_callback(self, msg):
        self.voltage = msg.data

def main():
    print("\n" + "="*60)
    print("  POWER DIAGNOSTIC - Yahboom Rosmaster R2")
    print("="*60 + "\n")

    rclpy.init()
    node = PowerChecker()

    print("Reading battery voltage...")

    # Wait for voltage reading
    for i in range(10):
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.voltage is not None:
            break

    if node.voltage is None:
        print("❌ ERROR: Cannot read voltage topic!")
        print("   The driver node may not be running.\n")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    print(f"\n📊 Battery Voltage: {node.voltage:.2f}V\n")

    # Analyze voltage
    if node.voltage < 0.5:
        print("❌ CRITICAL: NO BATTERY POWER DETECTED!")
        print("\n🔧 TROUBLESHOOTING STEPS:\n")
        print("1. CHECK BATTERY POWER SWITCH")
        print("   • Look for a power switch on the robot chassis")
        print("   • It should be in the ON position")
        print("   • Usually has a red LED when ON\n")

        print("2. CHECK BATTERY CONNECTION")
        print("   • Open the robot chassis")
        print("   • Verify battery connector is plugged into motor board")
        print("   • Check for loose connections\n")

        print("3. CHECK BATTERY CHARGE")
        print("   • Battery may be completely depleted")
        print("   • Remove battery and charge separately")
        print("   • Minimum voltage should be > 11V\n")

        print("4. CHECK MOTOR CONTROLLER BOARD")
        print("   • Look for power LED on the motor controller")
        print("   • Should be lit when battery is connected")
        print("   • Board may be faulty if no LED\n")

        print("⚠️  NOTE: Wall charger powers the Jetson but NOT the motors!")
        print("   Motors require battery power to function.\n")

    elif node.voltage < 11.0:
        print("⚠️  WARNING: Battery voltage is LOW!")
        print(f"   Current: {node.voltage:.2f}V")
        print("   Recommended: > 11.5V for proper operation")
        print("   Motors may not work properly at this voltage.\n")
        print("🔌 Please charge the battery fully.\n")

    elif node.voltage < 11.5:
        print("⚠️  Battery voltage is acceptable but could be higher")
        print(f"   Current: {node.voltage:.2f}V")
        print("   Optimal: 12.0V - 12.6V")
        print("   Motors should work but may have reduced power.\n")

    else:
        print("✅ Battery voltage is GOOD!")
        print(f"   Current: {node.voltage:.2f}V")
        print("   Motors should have full power.\n")
        print("   If motors still don't work, check:")
        print("   • Motor connections to controller board")
        print("   • Motor power fuse (if present)")
        print("   • Run: ros2 topic echo /cmd_vel\n")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
