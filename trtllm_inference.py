#!/usr/bin/env python3
"""
TensorRT-LLM Inference Wrapper for Qwen3 0.6B
Optimized for low-latency autonomous driving decisions on Jetson Orin
"""

import os
import sys
import time
import json
import torch
from pathlib import Path

# Add TensorRT-LLM to path if needed
trtllm_path = "/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/TensorRT-LLM"
if trtllm_path not in sys.path:
    sys.path.insert(0, trtllm_path)

import tensorrt_llm
from tensorrt_llm.runtime import ModelRunner
from tensorrt_llm.logger import logger


class TRTLLMInference:
    """
    TensorRT-LLM Inference wrapper for Qwen3 0.6B model

    Features:
    - Low-latency inference (<400ms target)
    - JSON mode support for structured outputs
    - Deterministic generation (temperature=0, seed=42)
    - Optimized for single-request, low-latency use case
    """

    def __init__(
        self,
        engine_dir: str,
        tokenizer_dir: str,
        max_output_len: int = 50,
        temperature: float = 0.0,
        top_k: int = 1,
        top_p: float = 1.0,
        seed: int = 42,
        use_mmap: bool = True,
        log_level: str = "error"
    ):
        """
        Initialize TensorRT-LLM inference engine

        Args:
            engine_dir: Path to TensorRT engine directory
            tokenizer_dir: Path to tokenizer directory (HuggingFace model dir)
            max_output_len: Maximum output tokens
            temperature: Sampling temperature (0.0 = deterministic)
            top_k: Top-K sampling
            top_p: Top-P sampling
            seed: Random seed for reproducibility
            use_mmap: Use mmap to reduce memory usage
            log_level: Logging level (error, warning, info, verbose)
        """
        self.engine_dir = Path(engine_dir)
        self.tokenizer_dir = Path(tokenizer_dir)
        self.max_output_len = max_output_len
        self.temperature = temperature
        self.top_k = top_k
        self.top_p = top_p
        self.seed = seed
        self.use_mmap = use_mmap

        # Set logging level
        logger.set_level(log_level)

        # Validate paths
        if not self.engine_dir.exists():
            raise ValueError(f"Engine directory not found: {engine_dir}")
        if not self.tokenizer_dir.exists():
            raise ValueError(f"Tokenizer directory not found: {tokenizer_dir}")

        # Load tokenizer
        print(f"Loading tokenizer from {self.tokenizer_dir}...")
        from transformers import AutoTokenizer
        self.tokenizer = AutoTokenizer.from_pretrained(
            str(self.tokenizer_dir),
            trust_remote_code=True,
            use_fast=True
        )

        # Get pad and end tokens
        self.pad_id = self.tokenizer.pad_token_id
        self.end_id = self.tokenizer.eos_token_id

        # Initialize ModelRunner
        print(f"Loading TensorRT-LLM engine from {self.engine_dir}...")
        start_time = time.time()

        self.runner = ModelRunner.from_dir(
            engine_dir=str(self.engine_dir),
            rank=tensorrt_llm.mpi_rank(),
            debug_mode=False
        )

        load_time = time.time() - start_time
        print(f"✓ TensorRT-LLM engine loaded in {load_time:.2f}s")

        # Set random seed for deterministic inference
        torch.manual_seed(self.seed)
        torch.cuda.manual_seed_all(self.seed)

    def generate(
        self,
        prompt: str,
        system_prompt: str = None,
        use_json_mode: bool = False
    ) -> tuple[str, float]:
        """
        Generate text completion for the given prompt

        Args:
            prompt: User prompt
            system_prompt: Optional system prompt
            use_json_mode: If True, instructs model to output JSON

        Returns:
            Tuple of (generated_text, latency_ms)
        """
        start_time = time.time()

        # Format prompt with system message if provided
        if system_prompt:
            # Qwen2.5 chat format
            full_prompt = f"<|im_start|>system\n{system_prompt}<|im_end|>\n<|im_start|>user\n{prompt}<|im_end|>\n<|im_start|>assistant\n"
        else:
            full_prompt = prompt

        # Add JSON instruction if needed
        if use_json_mode and "JSON" not in full_prompt:
            full_prompt += " Output in JSON format only."

        # Tokenize input
        input_ids = self.tokenizer.encode(
            full_prompt,
            add_special_tokens=False,
            return_tensors="pt"
        )

        # Prepare batch input (single request)
        batch_input_ids = [input_ids[0]]

        # Run inference
        with torch.no_grad():
            outputs = self.runner.generate(
                batch_input_ids=batch_input_ids,
                max_new_tokens=self.max_output_len,
                end_id=self.end_id,
                pad_id=self.pad_id,
                temperature=self.temperature,
                top_k=self.top_k,
                top_p=self.top_p,
                num_beams=1,  # Greedy decoding for lowest latency
                return_dict=True
            )

        # Decode output - skip input tokens to get only generated text
        output_ids = outputs['output_ids'][0][0]  # [batch_idx=0][beam_idx=0]
        input_length = len(batch_input_ids[0])

        # Extract only the newly generated tokens
        generated_ids = output_ids[input_length:]
        generated_text = self.tokenizer.decode(
            generated_ids,
            skip_special_tokens=True
        )

        # Calculate latency
        latency_ms = (time.time() - start_time) * 1000

        return generated_text.strip(), latency_ms

    def query_json(
        self,
        prompt: str,
        system_prompt: str = "You are an autonomous robot navigation AI. Output ONLY valid JSON with driving decisions. Be concise and deterministic."
    ) -> tuple[dict, float]:
        """
        Generate JSON response (convenience method for autonomous driving)

        Args:
            prompt: Navigation prompt
            system_prompt: System instruction

        Returns:
            Tuple of (parsed_json_dict, latency_ms)
        """
        response, latency = self.generate(
            prompt=prompt,
            system_prompt=system_prompt,
            use_json_mode=True
        )

        # Parse JSON - handle markdown code blocks
        import re

        # Strip markdown code blocks (```json ... ``` or ``` ... ```)
        cleaned = re.sub(r'```(?:json)?\s*', '', response).strip()

        try:
            result = json.loads(cleaned)
            return result, latency
        except json.JSONDecodeError as e:
            # Fallback: try to extract JSON from response
            json_match = re.search(r'\{.*\}', cleaned, re.DOTALL)
            if json_match:
                try:
                    result = json.loads(json_match.group())
                    return result, latency
                except:
                    pass

            # Return error structure
            return {
                "error": "JSON parse failed",
                "raw_response": response,
                "exception": str(e)
            }, latency


def benchmark_inference(engine_dir: str, tokenizer_dir: str, num_iterations: int = 100):
    """
    Benchmark inference latency

    Args:
        engine_dir: Path to TensorRT engine
        tokenizer_dir: Path to tokenizer
        num_iterations: Number of test iterations
    """
    print(f"\n{'='*60}")
    print("TensorRT-LLM Inference Benchmark")
    print(f"{'='*60}\n")

    # Initialize
    llm = TRTLLMInference(
        engine_dir=engine_dir,
        tokenizer_dir=tokenizer_dir,
        max_output_len=50,
        temperature=0.0,
        top_k=1,
        seed=42,
        use_mmap=True,
        log_level="error"
    )

    # Test prompts
    test_prompts = [
        "Robot sees clear path ahead. What action should it take?",
        "Obstacle detected 2 meters ahead on left. Clear path on right. Decision?",
        "Multiple obstacles detected. All paths blocked. What should robot do?",
    ]

    latencies = []

    print(f"Running {num_iterations} inferences...")
    for i in range(num_iterations):
        prompt = test_prompts[i % len(test_prompts)]

        result, latency = llm.query_json(prompt)
        latencies.append(latency)

        if i % 10 == 0:
            print(f"Iteration {i:3d}: {latency:6.1f}ms")

    # Calculate statistics
    import numpy as np
    latencies = np.array(latencies)

    print(f"\n{'='*60}")
    print("Latency Statistics")
    print(f"{'='*60}")
    print(f"Mean:    {np.mean(latencies):6.1f}ms")
    print(f"Median:  {np.median(latencies):6.1f}ms")
    print(f"Min:     {np.min(latencies):6.1f}ms")
    print(f"Max:     {np.max(latencies):6.1f}ms")
    print(f"P95:     {np.percentile(latencies, 95):6.1f}ms")
    print(f"P99:     {np.percentile(latencies, 99):6.1f}ms")
    print(f"{'='*60}\n")

    # Test JSON parsing
    print("Testing JSON output:")
    result, latency = llm.query_json("Robot sees obstacle ahead. Clear path on right. Decision?")
    print(f"Response ({latency:.1f}ms):")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="TensorRT-LLM Inference for Qwen3 0.6B")
    parser.add_argument("--engine_dir", required=True, help="TensorRT engine directory")
    parser.add_argument("--tokenizer_dir", required=True, help="Tokenizer directory")
    parser.add_argument("--benchmark", action="store_true", help="Run benchmark")
    parser.add_argument("--num_iterations", type=int, default=100, help="Benchmark iterations")
    parser.add_argument("--prompt", type=str, help="Single inference prompt")

    args = parser.parse_args()

    if args.benchmark:
        benchmark_inference(args.engine_dir, args.tokenizer_dir, args.num_iterations)
    elif args.prompt:
        llm = TRTLLMInference(
            engine_dir=args.engine_dir,
            tokenizer_dir=args.tokenizer_dir,
            log_level="info"
        )
        result, latency = llm.query_json(args.prompt)
        print(f"\nResponse ({latency:.1f}ms):")
        print(json.dumps(result, indent=2))
    else:
        print("Error: Specify --benchmark or --prompt")
        sys.exit(1)
