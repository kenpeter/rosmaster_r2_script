#!/bin/bash
# Convert Qwen3 0.6B to TensorRT-LLM engine
# Optimized for Jetson Orin with INT4 weight-only quantization

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Paths
WORKSPACE_DIR="/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws"
TRTLLM_DIR="$WORKSPACE_DIR/TensorRT-LLM"
MODEL_DIR="$WORKSPACE_DIR/models/qwen3-0.6b"
CHECKPOINT_DIR="$WORKSPACE_DIR/models/qwen3-trtllm-checkpoint"
ENGINE_DIR="$WORKSPACE_DIR/models/qwen3-trtllm-engine"

echo -e "${CYAN}============================================================${NC}"
echo -e "${CYAN}TensorRT-LLM Model Conversion for Qwen3 0.6B${NC}"
echo -e "${CYAN}============================================================${NC}"
echo ""

# Check if TensorRT-LLM is installed
if ! python3 -c "import tensorrt_llm" 2>/dev/null; then
    echo -e "${RED}✗ TensorRT-LLM not installed!${NC}"
    echo -e "${YELLOW}Please run: pip install TensorRT-LLM/build/tensorrt_llm-*.whl${NC}"
    exit 1
fi

echo -e "${GREEN}✓ TensorRT-LLM installed${NC}"

# Check if model exists
if [ ! -d "$MODEL_DIR" ]; then
    echo -e "${RED}✗ Model directory not found: $MODEL_DIR${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Model found: $MODEL_DIR${NC}"
echo ""

# Step 1: Convert checkpoint with INT4 weight-only quantization
echo -e "${CYAN}Step 1: Converting checkpoint to TensorRT-LLM format${NC}"
echo -e "${CYAN}  • Using INT4 weight-only quantization for low memory${NC}"
echo -e "${CYAN}  • FP16 activations for speed${NC}"
echo ""

cd "$TRTLLM_DIR/examples/qwen"

python3 convert_checkpoint.py \
    --model_dir "$MODEL_DIR" \
    --output_dir "$CHECKPOINT_DIR" \
    --dtype float16 \
    --use_weight_only \
    --weight_only_precision int4 \
    2>&1 | tee /tmp/qwen_convert_checkpoint.log

if [ ${PIPESTATUS[0]} -ne 0 ]; then
    echo -e "${RED}✗ Checkpoint conversion failed!${NC}"
    echo -e "${YELLOW}Check logs: /tmp/qwen_convert_checkpoint.log${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Checkpoint converted successfully${NC}"
echo ""

# Step 2: Build TensorRT engine
echo -e "${CYAN}Step 2: Building TensorRT engine${NC}"
echo -e "${CYAN}  • GEMM plugin: float16${NC}"
echo -e "${CYAN}  • Target: Jetson Orin (CUDA arch 8.7)${NC}"
echo -e "${CYAN}  • Estimated time: 10-15 minutes${NC}"
echo ""

# Add trtllm-build to PATH if needed
export PATH="$HOME/.local/bin:$PATH"

trtllm-build \
    --checkpoint_dir "$CHECKPOINT_DIR" \
    --output_dir "$ENGINE_DIR" \
    --gemm_plugin float16 \
    --max_batch_size 1 \
    --max_input_len 512 \
    --max_output_len 128 \
    2>&1 | tee /tmp/qwen_build_engine.log

if [ ${PIPESTATUS[0]} -ne 0 ]; then
    echo -e "${RED}✗ Engine build failed!${NC}"
    echo -e "${YELLOW}Check logs: /tmp/qwen_build_engine.log${NC}"
    exit 1
fi

echo -e "${GREEN}✓ TensorRT engine built successfully${NC}"
echo ""

# Step 3: Verify engine
echo -e "${CYAN}Step 3: Verifying engine${NC}"
echo ""

ls -lh "$ENGINE_DIR"
echo ""

echo -e "${GREEN}✓ Engine files created${NC}"
echo ""

# Step 4: Test inference
echo -e "${CYAN}Step 4: Testing inference${NC}"
echo ""

cd "$TRTLLM_DIR/examples"

python3 run.py \
    --input_text "Robot sees clear path ahead. What should it do?" \
    --max_output_len 50 \
    --tokenizer_dir "$MODEL_DIR" \
    --engine_dir "$ENGINE_DIR" \
    --use_mmap \
    2>&1 | tee /tmp/qwen_test_inference.log

if [ ${PIPESTATUS[0]} -ne 0 ]; then
    echo -e "${YELLOW}Warning: Test inference had issues${NC}"
    echo -e "${YELLOW}Check logs: /tmp/qwen_test_inference.log${NC}"
else
    echo -e "${GREEN}✓ Test inference successful${NC}"
fi

echo ""
echo -e "${CYAN}============================================================${NC}"
echo -e "${GREEN}TensorRT-LLM Model Conversion Complete!${NC}"
echo -e "${CYAN}============================================================${NC}"
echo ""
echo -e "${CYAN}Next steps:${NC}"
echo -e "  1. Run benchmark: python3 scripts/trtllm_inference.py --engine_dir $ENGINE_DIR --tokenizer_dir $MODEL_DIR --benchmark"
echo -e "  2. Update ROS node: scripts/update_llm_node_trtllm.sh"
echo -e "  3. Test autonomous system: ./scripts/start_auto.py"
echo ""
