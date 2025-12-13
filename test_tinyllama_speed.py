#!/usr/bin/env python3
"""
TinyLlama Inference Speed Test
Tests if TinyLlama 1.1B meets the 0.3-0.5s inference requirement
for the autonomous driving system (target: 1.0-1.5 Hz control loop)
"""

import subprocess
import time
import json

def test_ollama_inference(prompt, num_runs=5):
    """Test Ollama inference speed with TinyLlama"""
    print(f"\n🧪 Testing TinyLlama inference speed...")
    print(f"   Prompt: '{prompt}'")
    print(f"   Runs: {num_runs}\n")

    times = []

    for i in range(num_runs):
        print(f"Run {i+1}/{num_runs}...", end=" ")

        start_time = time.time()

        # Run ollama with JSON output
        result = subprocess.run(
            ["ollama", "run", "tinyllama:1.1b", prompt],
            capture_output=True,
            text=True,
            timeout=10
        )

        elapsed = time.time() - start_time
        times.append(elapsed)

        print(f"{elapsed:.3f}s")

        # Brief pause between runs
        if i < num_runs - 1:
            time.sleep(0.5)

    return times

def main():
    print("="*60)
    print("  TinyLlama 1.1B Inference Speed Test")
    print("="*60)
    print("\nTarget: 0.3-0.5s per inference")
    print("Control loop: 1.0-1.5 Hz (0.67-1.0s total cycle time)")
    print("\nTest scenarios:")

    # Test 1: Short navigation decision
    print("\n" + "-"*60)
    print("[TEST 1] Short Navigation Decision")
    print("-"*60)
    times_short = test_ollama_inference(
        "Navigate: obstacle ahead. Turn left or right? Answer in 3 words.",
        num_runs=3
    )

    # Test 2: Medium reasoning task
    print("\n" + "-"*60)
    print("[TEST 2] Medium Reasoning Task")
    print("-"*60)
    times_medium = test_ollama_inference(
        "Robot sees: person 2m ahead, car 5m left. Should I stop or slow down? One sentence.",
        num_runs=3
    )

    # Calculate statistics
    all_times = times_short + times_medium
    avg_time = sum(all_times) / len(all_times)
    min_time = min(all_times)
    max_time = max(all_times)

    print("\n" + "="*60)
    print("  RESULTS")
    print("="*60)
    print(f"\n📊 Statistics:")
    print(f"   Average: {avg_time:.3f}s")
    print(f"   Min:     {min_time:.3f}s")
    print(f"   Max:     {max_time:.3f}s")

    print(f"\n🎯 Target Analysis:")
    if avg_time <= 0.5:
        print(f"   ✅ PASS - Average {avg_time:.3f}s ≤ 0.5s target")
        print(f"   ✅ Supports 1.0-1.5 Hz control loop")
    elif avg_time <= 0.7:
        print(f"   ⚠️  MARGINAL - Average {avg_time:.3f}s slightly over 0.5s")
        print(f"   ⚠️  May support 1.0 Hz with optimizations")
    else:
        print(f"   ❌ FAIL - Average {avg_time:.3f}s > 0.7s")
        print(f"   ❌ Too slow for 1.0-1.5 Hz control loop")

    print(f"\n💡 Recommendations:")
    if avg_time <= 0.5:
        print("   • TinyLlama is fast enough for real-time control")
        print("   • Can run reasoning at 1.5-2 Hz")
        print("   • Consider GPU acceleration for even faster inference")
    else:
        print("   • Consider prompt optimization (shorter prompts)")
        print("   • Enable GPU acceleration if not already active")
        print("   • May need to reduce control loop frequency")

    print("\n" + "="*60)

if __name__ == "__main__":
    main()
