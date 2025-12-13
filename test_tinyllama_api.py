#!/usr/bin/env python3
"""
TinyLlama API Inference Speed Test
Uses Ollama HTTP API for faster inference (bypasses CLI overhead)
Tests realistic inference speed for continuous autonomous operation
"""

import requests
import time
import json

OLLAMA_API = "http://localhost:11434/api/generate"
MODEL = "tinyllama:1.1b"

def warmup_model():
    """Warm up the model with a dummy inference"""
    print("🔥 Warming up model...")
    response = requests.post(
        OLLAMA_API,
        json={
            "model": MODEL,
            "prompt": "Hello",
            "stream": False
        }
    )
    print("   ✅ Model ready\n")

def test_inference_api(prompt, num_runs=10):
    """Test inference speed using Ollama API"""
    times = []
    responses = []

    for i in range(num_runs):
        start_time = time.time()

        response = requests.post(
            OLLAMA_API,
            json={
                "model": MODEL,
                "prompt": prompt,
                "stream": False
            }
        )

        elapsed = time.time() - start_time
        times.append(elapsed)

        if response.status_code == 200:
            data = response.json()
            responses.append(data.get('response', ''))
            print(f"Run {i+1:2d}/{num_runs}: {elapsed:.3f}s - {data.get('response', '')[:50]}")
        else:
            print(f"Run {i+1:2d}/{num_runs}: {elapsed:.3f}s - ERROR")

    return times, responses

def main():
    print("="*60)
    print("  TinyLlama API Inference Speed Test")
    print("="*60)
    print("\nTarget: 0.3-0.5s per inference")
    print("Control loop: 1.0-1.5 Hz (0.67-1.0s cycle)")
    print(f"Model: {MODEL}")
    print(f"API: {OLLAMA_API}\n")

    # Warm up
    warmup_model()

    # Test 1: Ultra-short prompts (navigation decisions)
    print("-"*60)
    print("[TEST 1] Ultra-Short Navigation Prompts")
    print("-"*60)
    times_short, _ = test_inference_api(
        "Turn left or right?",
        num_runs=10
    )

    # Test 2: Short reasoning (realistic autonomous driving)
    print("\n" + "-"*60)
    print("[TEST 2] Short Reasoning Prompts")
    print("-"*60)
    times_medium, _ = test_inference_api(
        "Obstacle ahead 2m. Action?",
        num_runs=10
    )

    # Calculate statistics (exclude first run warmup)
    all_times = times_short[1:] + times_medium[1:]  # Skip first run
    avg_time = sum(all_times) / len(all_times)
    min_time = min(all_times)
    max_time = max(all_times)
    median_time = sorted(all_times)[len(all_times)//2]

    print("\n" + "="*60)
    print("  RESULTS (excluding first warmup run)")
    print("="*60)
    print(f"\n📊 Statistics:")
    print(f"   Average: {avg_time:.3f}s")
    print(f"   Median:  {median_time:.3f}s")
    print(f"   Min:     {min_time:.3f}s")
    print(f"   Max:     {max_time:.3f}s")

    # Calculate achievable frequency
    cycle_time = avg_time
    frequency = 1.0 / cycle_time if cycle_time > 0 else 0

    print(f"\n🎯 Performance Analysis:")
    print(f"   Inference time: {avg_time:.3f}s")
    print(f"   Max frequency:  {frequency:.2f} Hz")

    if avg_time <= 0.5:
        print(f"   ✅ EXCELLENT - Meets 0.3-0.5s target!")
        print(f"   ✅ Supports {frequency:.1f} Hz LLM updates")
        if frequency >= 2.0:
            print(f"   ✅ Can run at 2+ Hz for fast decision making")
    elif avg_time <= 0.7:
        print(f"   ⚠️  GOOD - Slightly over target but usable")
        print(f"   ✅ Supports {frequency:.1f} Hz control loop")
    else:
        print(f"   ❌ SLOW - {avg_time:.3f}s > 0.7s target")
        print(f"   ⚠️  Only {frequency:.1f} Hz possible")

    print(f"\n💡 Autonomous Driving Feasibility:")
    if avg_time <= 0.5:
        print("   ✅ Real-time control: FEASIBLE")
        print("   ✅ Can run perception (0.1-0.2s) + LLM (0.3-0.5s)")
        print("   ✅ Total cycle: 0.4-0.7s = 1.4-2.5 Hz")
    else:
        print("   ⚠️  Real-time control: MARGINAL")
        print("   ⚠️  Consider async LLM updates (not every cycle)")

    print("\n" + "="*60)

if __name__ == "__main__":
    main()
