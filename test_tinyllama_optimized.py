#!/usr/bin/env python3
"""
TinyLlama OPTIMIZED Inference Speed Test
Uses token limits and concise prompts for autonomous driving
"""

import requests
import time
import json

OLLAMA_API = "http://localhost:11434/api/generate"
MODEL = "tinyllama:1.1b"

def warmup_model():
    """Warm up the model"""
    print("🔥 Warming up model...")
    response = requests.post(
        OLLAMA_API,
        json={
            "model": MODEL,
            "prompt": "Hi",
            "stream": False,
            "options": {"num_predict": 5}
        }
    )
    print("   ✅ Model ready\n")

def test_inference_optimized(prompt, max_tokens=10, num_runs=10):
    """Test inference with token limits"""
    times = []
    responses = []

    for i in range(num_runs):
        start_time = time.time()

        response = requests.post(
            OLLAMA_API,
            json={
                "model": MODEL,
                "prompt": prompt,
                "stream": False,
                "options": {
                    "num_predict": max_tokens,  # Limit output tokens
                    "temperature": 0.1,         # More deterministic
                    "top_p": 0.9
                }
            }
        )

        elapsed = time.time() - start_time
        times.append(elapsed)

        if response.status_code == 200:
            data = response.json()
            resp_text = data.get('response', '').strip()
            responses.append(resp_text)
            print(f"Run {i+1:2d}/{num_runs}: {elapsed:.3f}s → '{resp_text}'")
        else:
            print(f"Run {i+1:2d}/{num_runs}: {elapsed:.3f}s → ERROR")

    return times, responses

def main():
    print("="*70)
    print("  TinyLlama OPTIMIZED Inference Test")
    print("="*70)
    print("\n🎯 Target: 0.3-0.5s for autonomous driving (1.0-1.5 Hz control)")
    print(f"📦 Model: {MODEL}")
    print("⚡ Optimization: Token limits + concise prompts\n")

    warmup_model()

    # Test 1: Binary decision (left/right)
    print("-"*70)
    print("[TEST 1] Binary Decision - 5 tokens max")
    print("-"*70)
    times_binary, resp_binary = test_inference_optimized(
        "Obstacle ahead. LEFT or RIGHT?",
        max_tokens=5,
        num_runs=10
    )

    # Test 2: Action command - 10 tokens max
    print("\n" + "-"*70)
    print("[TEST 2] Action Command - 10 tokens max")
    print("-"*70)
    times_action, resp_action = test_inference_optimized(
        "Person 2m ahead. STOP, SLOW, or GO?",
        max_tokens=10,
        num_runs=10
    )

    # Test 3: Speed decision - 8 tokens max
    print("\n" + "-"*70)
    print("[TEST 3] Speed Decision - 8 tokens max")
    print("-"*70)
    times_speed, resp_speed = test_inference_optimized(
        "Clear road. Speed: FAST, MEDIUM, or SLOW?",
        max_tokens=8,
        num_runs=10
    )

    # Calculate statistics (exclude first run from each test)
    all_times = times_binary[1:] + times_action[1:] + times_speed[1:]
    avg_time = sum(all_times) / len(all_times)
    min_time = min(all_times)
    max_time = max(all_times)
    median_time = sorted(all_times)[len(all_times)//2]

    # Calculate percentiles
    sorted_times = sorted(all_times)
    p50 = sorted_times[len(sorted_times)//2]
    p95 = sorted_times[int(len(sorted_times)*0.95)]
    p99 = sorted_times[int(len(sorted_times)*0.99)]

    print("\n" + "="*70)
    print("  FINAL RESULTS (27 runs, excluding warmup)")
    print("="*70)
    print(f"\n📊 Statistics:")
    print(f"   Average:     {avg_time:.3f}s")
    print(f"   Median:      {median_time:.3f}s")
    print(f"   Min:         {min_time:.3f}s")
    print(f"   Max:         {max_time:.3f}s")
    print(f"   95th %ile:   {p95:.3f}s")
    print(f"   99th %ile:   {p99:.3f}s")

    # Frequency calculation
    frequency = 1.0 / avg_time if avg_time > 0 else 0

    print(f"\n⚡ Performance:")
    print(f"   LLM inference:   {avg_time:.3f}s")
    print(f"   Max frequency:   {frequency:.2f} Hz")

    print(f"\n🎯 Target Analysis:")
    if avg_time <= 0.5:
        status = "✅ EXCELLENT"
        feasibility = "FULLY FEASIBLE"
    elif avg_time <= 0.7:
        status = "✅ GOOD"
        feasibility = "FEASIBLE"
    elif avg_time <= 1.0:
        status = "⚠️  ACCEPTABLE"
        feasibility = "POSSIBLE with optimizations"
    else:
        status = "❌ SLOW"
        feasibility = "NOT FEASIBLE for real-time"

    print(f"   Status: {status}")
    print(f"   {avg_time:.3f}s vs 0.3-0.5s target")

    print(f"\n🤖 Autonomous Driving Analysis:")
    print(f"   Real-time control: {feasibility}")

    # Calculate total cycle time
    perception_time = 0.15  # YOLO + DINOv2
    llm_time = avg_time
    control_time = 0.05     # Motor commands
    total_cycle = perception_time + llm_time + control_time

    print(f"\n📈 Full Control Loop Estimate:")
    print(f"   Perception (YOLO+DINOv2): ~0.15s")
    print(f"   LLM reasoning:            {avg_time:.2f}s")
    print(f"   Control output:           ~0.05s")
    print(f"   ─────────────────────────────────")
    print(f"   Total cycle time:         ~{total_cycle:.2f}s")
    print(f"   Control frequency:        ~{1.0/total_cycle:.2f} Hz")

    if total_cycle <= 1.0:
        print(f"   ✅ Meets 1.0 Hz minimum target!")
    else:
        print(f"   ⚠️  Below 1.0 Hz target")

    print("\n💡 Recommendations:")
    if avg_time <= 0.5:
        print("   ✅ Token limiting works! Keep max_tokens=5-10")
        print("   ✅ Use concise prompts with clear options")
        print("   ✅ Consider running LLM at {:.1f} Hz for fast decisions".format(frequency))
    else:
        print("   ⚠️  Consider async architecture:")
        print("      - Run perception at 5-10 Hz")
        print("      - Run LLM at {:.1f} Hz (not every frame)".format(frequency))
        print("      - Cache decisions for intermediate frames")

    print("\n" + "="*70)

if __name__ == "__main__":
    main()
