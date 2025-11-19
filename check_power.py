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

    print(f"\n📊 Battery Voltage Sensor Reading: {node.voltage:.2f}V\n")

    # Analyze voltage for Motors + YDLidar power requirements
    if node.voltage < 0.5:
        print("⚠️  VOLTAGE SENSOR ISSUE DETECTED!\n")
        print("📋 SITUATION ANALYSIS:")
        print("   • Jetson is running (powered ON) ✅")
        print("   • Motor board is communicating ✅")
        print("   • But voltage sensor reads: {:.2f}V ❌\n".format(node.voltage))
        print("🔍 This means:")
        print("   • Battery IS providing power to the system")
        print("   • But the voltage SENSOR is broken/disconnected")
        print("   • Cannot determine actual battery level\n")
        print("⚠️  RISKS:")
        print("   • Battery may be LOW without warning")
        print("   • System could shut down unexpectedly")
        print("   • Motors/YDLidar may fail due to voltage sag\n")
        print("🔧 RECOMMENDATIONS:")
        print("   1. Test motors: ros2 topic pub /cmd_vel ...")
        print("   2. If motors are WEAK or DON'T WORK:")
        print("      → Battery voltage is too LOW, charge immediately")
        print("   3. If motors work NORMALLY:")
        print("      → Battery has sufficient charge")
        print("      → But voltage sensor needs repair\n")
        print("   4. Use multimeter to check battery directly:")
        print("      → Should read 11.5V - 12.6V for proper operation\n")

    elif node.voltage < 11.5:
        print("❌ INSUFFICIENT POWER: Battery voltage TOO LOW!")
        print(f"   Current: {node.voltage:.2f}V")
        print(f"   Required: ≥ 11.5V for Motors + YDLidar\n")
        print("⚠️  IMPACT:")
        print("   • Motors will NOT work reliably")
        print("   • YDLidar may fail or produce errors")
        print("   • Combined power draw will cause voltage sag")
        print("   • Robot may behave erratically\n")
        print("🔌 ACTION REQUIRED: Charge battery immediately!")
        print("   Target voltage: 12.0V - 12.6V (fully charged)\n")

    elif node.voltage < 12.0:
        print("⚠️  MARGINAL POWER: Battery voltage is ACCEPTABLE")
        print(f"   Current: {node.voltage:.2f}V")
        print(f"   Optimal: 12.0V - 12.6V\n")
        print("✅ Motors should work")
        print("✅ YDLidar should work")
        print("⚠️  However:")
        print("   • Battery will drain faster under load")
        print("   • Performance may degrade during operation")
        print("   • Consider charging soon for best results\n")

    else:
        print("✅ EXCELLENT: Battery has SUFFICIENT POWER!")
        print(f"   Current: {node.voltage:.2f}V")
        print(f"   Status: Optimal for full operation\n")
        print("✅ Motors: Full power available")
        print("✅ YDLidar: Full power available")
        print("✅ Combined load: Battery can handle it\n")
        print("   If motors/lidar still don't work, check:")
        print("   • Motor connections to controller board")
        print("   • YDLidar USB connection")
        print("   • Run: ros2 topic list\n")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
