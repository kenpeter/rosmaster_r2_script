#!/usr/bin/env python3
"""
Battery Level Test - Estimates voltage by testing motor response
Since voltage sensor is broken (reads 0.09V), this tests motor behavior
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

print("\n" + "="*60)
print("  BATTERY VOLTAGE ESTIMATION TEST")
print("="*60)
print("\nSince voltage sensor is broken, testing motors to estimate")
print("battery charge level...\n")

rclpy.init()
node = Node('battery_tester')
pub = node.create_publisher(Twist, '/cmd_vel', 10)

# Wait for publisher to connect
time.sleep(1)

print("🧪 RUNNING MOTOR TESTS...\n")
print("=" * 60)

# Test 1: Low speed forward
print("\n[TEST 1/3] Low Speed Forward (20%)")
print("Watch the motors closely...")
twist = Twist()
twist.linear.x = 0.2
pub.publish(twist)
time.sleep(2)

# Stop
twist.linear.x = 0.0
pub.publish(twist)
time.sleep(1)

# Test 2: Medium speed forward
print("\n[TEST 2/3] Medium Speed Forward (50%)")
print("Watch the motors closely...")
twist.linear.x = 0.5
pub.publish(twist)
time.sleep(2)

# Stop
twist.linear.x = 0.0
pub.publish(twist)
time.sleep(1)

# Test 3: High speed forward
print("\n[TEST 3/3] High Speed Forward (80%)")
print("Watch the motors closely...")
twist.linear.x = 0.8
pub.publish(twist)
time.sleep(2)

# Stop
twist.linear.x = 0.0
twist.angular.z = 0.0
pub.publish(twist)

print("\n" + "="*60)
print("  TEST COMPLETE - EVALUATE RESULTS")
print("="*60 + "\n")

print("Based on what you observed:\n")

print("❌ MOTORS DID NOT MOVE AT ALL:")
print("   → Battery voltage: < 11.0V (TOO LOW)")
print("   → Status: NEEDS MORE CHARGING")
print("   → Action: Charge for 1-2 more hours\n")

print("⚠️  MOTORS MOVED WEAKLY/STUTTERED:")
print("   → Battery voltage: ~11.0V - 11.4V (MARGINAL)")
print("   → Status: ALMOST READY")
print("   → Action: Charge for 30-60 more minutes\n")

print("✅ MOTORS MOVED SMOOTHLY AT ALL SPEEDS:")
print("   → Battery voltage: ≥ 11.5V (GOOD)")
print("   → Status: READY FOR USE")
print("   → Motors + YDLidar should work normally\n")

print("🔋 NOTE: Fully charged battery = 12.0V - 12.6V")
print("         You can continue charging for maximum runtime.\n")

node.destroy_node()
rclpy.shutdown()
