#!/usr/bin/env python3
"""
Vision-Language-Action Pipeline
YOLO11 + DINOv2 + TinyLlama - Complete AI Vision System
Sees, Understands, and Suggests Actions
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import torch
import os
import subprocess
import sys
from collections import deque

class VisionLLMNode(Node):
    def __init__(self):
        super().__init__('vision_llm_live')

        self.get_logger().info("=" * 70)
        self.get_logger().info("  🤖 Vision-Language-Action AI Pipeline")
        self.get_logger().info("=" * 70)
        self.get_logger().info("")

        # Load YOLO11 model
        self.get_logger().info("📦 [1/3] Loading YOLO11 model...")
        model_path = os.path.expanduser("~/yahboomcar_ros2_ws/yahboomcar_ws/models/yolo11s.pt")

        try:
            from ultralytics import YOLO
            self.yolo_model = YOLO(model_path)
            self.get_logger().info("✅ YOLO11-Small loaded")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load YOLO11: {e}")
            raise

        # Load DINOv2 model
        self.get_logger().info("🦖 [2/3] Loading DINOv2 model...")

        try:
            from transformers import AutoImageProcessor, AutoModel

            model_name = 'facebook/dinov2-small'
            self.dino_processor = AutoImageProcessor.from_pretrained(model_name)
            self.dino_model = AutoModel.from_pretrained(model_name)
            self.get_logger().info("✅ DINOv2-Small loaded")

            # Move to GPU if available
            if torch.cuda.is_available():
                self.dino_model = self.dino_model.cuda()

            self.dino_model.eval()

        except Exception as e:
            self.get_logger().error(f"❌ Failed to load DINOv2: {e}")
            raise

        # Load TinyLlama LLM
        self.get_logger().info("🧠 [3/3] Loading TinyLlama-1.1B LLM...")

        try:
            from transformers import AutoTokenizer, AutoModelForCausalLM

            llm_name = "TinyLlama/TinyLlama-1.1B-Chat-v1.0"
            self.tokenizer = AutoTokenizer.from_pretrained(llm_name)
            self.llm_model = AutoModelForCausalLM.from_pretrained(
                llm_name,
                torch_dtype=torch.float16 if torch.cuda.is_available() else torch.float32,
                low_cpu_mem_usage=True,
                attn_implementation="eager"  # Fix for PyTorch/Transformers compatibility
            )

            if torch.cuda.is_available():
                self.llm_model = self.llm_model.cuda()
                self.get_logger().info(f"🚀 Using GPU: {torch.cuda.get_device_name(0)}")
            else:
                self.get_logger().info("⚠️  Using CPU (slower)")

            self.llm_model.eval()
            self.get_logger().info("✅ TinyLlama loaded")

        except Exception as e:
            self.get_logger().error(f"❌ Failed to load LLM: {e}")
            raise

        # Detection parameters
        self.conf_threshold = 0.5
        self.heatmap_alpha = 0.3

        # LLM response cache (update every N frames to save compute)
        self.llm_response = "Initializing AI vision system..."
        self.llm_action = "Waiting for camera input..."
        self.frames_since_llm_update = 0
        self.llm_update_interval = 15  # Update LLM every 15 frames (~0.5-1 second)

        # CV Bridge
        self.bridge = CvBridge()

        # Publisher for visualization
        self.image_pub = self.create_publisher(Image, '/vision_llm/output', 10)

        # Try to subscribe to camera topic
        self.camera_available = False
        self.get_logger().info("📷 Looking for camera topic...")

        time.sleep(1)

        # Check if camera topic exists
        topic_list = self.get_topic_names_and_types()
        camera_topics = ['/camera/color/image_raw', '/camera/image_raw', '/image_raw', '/RGBD/RGB/Image']

        for topic in camera_topics:
            if any(topic in t[0] for t in topic_list):
                self.get_logger().info(f"✅ Found camera topic: {topic}")
                self.camera_sub = self.create_subscription(
                    Image,
                    topic,
                    self.camera_callback,
                    10
                )
                self.camera_available = True
                break

        if not self.camera_available:
            self.get_logger().error("❌ No camera topic found!")
            self.get_logger().error("")
            self.get_logger().error("To use real camera, you must start the robot hardware first:")
            self.get_logger().error("  1. Open a new terminal")
            self.get_logger().error("  2. Run: ./scripts/start_robot.sh")
            self.get_logger().error("  3. Then run this script again")
            self.get_logger().error("")
            raise RuntimeError("Camera not available. Please start robot hardware first.")

        # Performance tracking
        self.fps_history = deque(maxlen=30)
        self.last_log_time = time.time()

        # Detection history for LLM context
        self.recent_detections = deque(maxlen=5)

        self.get_logger().info("")
        self.get_logger().info("=" * 70)
        self.get_logger().info("  🎬 Vision-Language-Action Pipeline Started")
        self.get_logger().info("=" * 70)
        self.get_logger().info("")

    def camera_callback(self, msg):
        """Process real camera frame"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_frame(frame)
        except Exception as e:
            self.get_logger().error(f"Error processing camera frame: {e}")

    def process_frame(self, frame):
        """Run complete vision-language-action pipeline"""
        total_start_time = time.time()

        # 1. Run DINOv2 for attention heatmap
        attention_map = self.get_dino_attention(frame)

        # 2. Run YOLO for object detection
        yolo_results = self.yolo_model(frame, conf=self.conf_threshold, verbose=False)

        # 3. Extract detection information
        detections = self.extract_detections(yolo_results)
        self.recent_detections.append(detections)

        # 4. Update LLM response periodically
        self.frames_since_llm_update += 1
        if self.frames_since_llm_update >= self.llm_update_interval:
            self.update_llm_response(detections, attention_map)
            self.frames_since_llm_update = 0

        # 5. Create visualization with LLM overlay
        final_frame = self.create_complete_visualization(
            frame, attention_map, yolo_results, detections
        )

        # Calculate FPS
        total_time = (time.time() - total_start_time) * 1000  # ms
        fps = 1000 / total_time if total_time > 0 else 0
        self.fps_history.append(fps)
        avg_fps = sum(self.fps_history) / len(self.fps_history)

        # Publish visualization
        try:
            msg = self.bridge.cv2_to_imgmsg(final_frame, encoding='bgr8')
            self.image_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish image: {e}")

        # Log stats every 2 seconds
        current_time = time.time()
        if current_time - self.last_log_time > 2.0:
            self.get_logger().info(
                f"📊 {avg_fps:.1f} FPS | Objects: {len(detections)} | Action: {self.llm_action}"
            )
            self.last_log_time = current_time

    def get_dino_attention(self, frame):
        """Extract attention map from DINOv2"""
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image_resized = cv2.resize(frame_rgb, (224, 224))

        with torch.no_grad():
            inputs = self.dino_processor(images=image_resized, return_tensors="pt")

            if torch.cuda.is_available():
                inputs = {k: v.cuda() for k, v in inputs.items()}

            outputs = self.dino_model(**inputs)
            features = outputs.last_hidden_state

        # Create attention map
        attention = features[0, 1:, :].mean(dim=-1)  # Skip CLS token
        num_patches = int(np.sqrt(attention.shape[0]))

        attention_map = attention.reshape(num_patches, num_patches)
        attention_map = attention_map.cpu().numpy()

        # Normalize to 0-255
        attention_map = (attention_map - attention_map.min()) / (attention_map.max() - attention_map.min() + 1e-8)
        attention_map = (attention_map * 255).astype(np.uint8)

        # Resize to match original frame
        h, w = frame.shape[:2]
        attention_resized = cv2.resize(attention_map, (w, h), interpolation=cv2.INTER_CUBIC)

        return attention_resized

    def extract_detections(self, yolo_results):
        """Extract structured detection information"""
        detections = []

        if len(yolo_results) > 0 and len(yolo_results[0].boxes) > 0:
            for box in yolo_results[0].boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = self.yolo_model.names[cls]
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                detections.append({
                    'class': class_name,
                    'confidence': conf,
                    'bbox': (x1, y1, x2, y2)
                })

        return detections

    def update_llm_response(self, detections, attention_map):
        """Generate LLM response based on vision input"""
        try:
            # Create prompt describing what the AI sees
            prompt = self.create_vision_prompt(detections, attention_map)

            # Format for TinyLlama chat template
            messages = [
                {"role": "system", "content": "You are a helpful robot assistant that describes what you see and suggests actions. Be concise and practical."},
                {"role": "user", "content": prompt}
            ]

            # Apply chat template
            formatted_prompt = self.tokenizer.apply_chat_template(
                messages,
                tokenize=False,
                add_generation_prompt=True
            )

            # Generate response
            inputs = self.tokenizer(formatted_prompt, return_tensors="pt", truncation=True, max_length=512)
            if torch.cuda.is_available():
                inputs = {k: v.cuda() for k, v in inputs.items()}

            with torch.no_grad():
                outputs = self.llm_model.generate(
                    **inputs,
                    max_new_tokens=80,
                    temperature=0.7,
                    do_sample=True,
                    top_p=0.9,
                    pad_token_id=self.tokenizer.eos_token_id,
                    eos_token_id=self.tokenizer.eos_token_id
                )

            response = self.tokenizer.decode(outputs[0], skip_special_tokens=True)

            # Extract just the assistant's response
            if "<|assistant|>" in response:
                response = response.split("<|assistant|>")[-1].strip()
            elif "user" in response.lower():
                # Try to get text after the last user prompt
                parts = response.split('\n')
                response = '\n'.join(parts[-3:]).strip()

            # Parse response into description and action
            self.parse_llm_response(response)

            # Log successful LLM update
            self.get_logger().info(f"LLM: {self.llm_action}")

        except Exception as e:
            self.get_logger().error(f"LLM generation error: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.llm_response = "LLM processing error"
            self.llm_action = "Monitoring scene..."

    def create_vision_prompt(self, detections, attention_map):
        """Create text prompt describing visual scene"""
        # Summarize detections
        if len(detections) == 0:
            scene = "I see an empty scene with no objects detected."
        else:
            object_counts = {}
            for det in detections:
                obj = det['class']
                object_counts[obj] = object_counts.get(obj, 0) + 1

            object_list = [f"{count} {obj}{'s' if count > 1 else ''}" for obj, count in object_counts.items()]
            scene = f"I see: {', '.join(object_list)}."

        # Attention info
        attention_mean = attention_map.mean()
        attention_max = attention_map.max()

        if attention_max > 200:
            attention_desc = "high visual complexity with distinct features"
        elif attention_max > 150:
            attention_desc = "moderate visual activity"
        else:
            attention_desc = "simple scene with few salient features"

        prompt = f"""Scene Analysis:
{scene}
Visual complexity: {attention_desc}.

As a robot, provide:
1. Brief scene description (one sentence)
2. Suggested action (one sentence, be specific)

Format your response as:
Description: [your description]
Action: [your suggested action]"""

        return prompt

    def parse_llm_response(self, response):
        """Parse LLM response into description and action"""
        lines = response.split('\n')

        description = ""
        action = ""

        for line in lines:
            line = line.strip()
            if line.startswith("Description:"):
                description = line.replace("Description:", "").strip()
            elif line.startswith("Action:"):
                action = line.replace("Action:", "").strip()

        # Fallback if parsing fails
        if not description and not action:
            parts = response.split('.')
            if len(parts) >= 2:
                description = parts[0].strip() + '.'
                action = parts[1].strip() + '.'
            else:
                description = response[:80] + "..." if len(response) > 80 else response
                action = "Continue monitoring"

        self.llm_response = description if description else "Processing scene..."
        self.llm_action = action if action else "Standby"

    def create_complete_visualization(self, frame, attention_map, yolo_results, detections):
        """Create complete visualization with vision + LLM overlay"""
        h, w = frame.shape[:2]

        # 1. Apply DINOv2 heatmap
        attention_colored = cv2.applyColorMap(attention_map, cv2.COLORMAP_JET)
        blended = cv2.addWeighted(frame, 1 - self.heatmap_alpha, attention_colored, self.heatmap_alpha, 0)

        # 2. Draw YOLO detections
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            class_name = det['class']
            conf = det['confidence']

            # Draw bounding box (green)
            cv2.rectangle(blended, (x1, y1), (x2, y2), (0, 255, 0), 3)

            # Draw label
            label = f"{class_name} {conf:.2f}"
            label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)

            cv2.rectangle(blended,
                        (x1, y1 - label_size[1] - 12),
                        (x1 + label_size[0] + 8, y1),
                        (0, 255, 0), -1)

            cv2.putText(blended, label, (x1 + 4, y1 - 6),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

        # 3. Draw LLM response panel at bottom
        panel_height = 180
        panel = np.zeros((panel_height, w, 3), dtype=np.uint8)
        panel[:] = (20, 20, 20)  # Dark gray background

        # Draw border
        cv2.rectangle(panel, (0, 0), (w-1, panel_height-1), (0, 255, 255), 2)

        # Title
        cv2.putText(panel, "AI REASONING:", (15, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # Description
        desc_wrapped = self.wrap_text(self.llm_response, 80)
        y_offset = 60
        for line in desc_wrapped[:2]:  # Max 2 lines
            cv2.putText(panel, line, (15, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            y_offset += 30

        # Action
        cv2.putText(panel, f"ACTION: {self.llm_action}", (15, y_offset + 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

        # 4. Combine vision frame and LLM panel
        final_frame = np.vstack([blended, panel])

        # 5. Add top stats overlay
        stats_bg = np.zeros((50, w, 3), dtype=np.uint8)
        stats_bg[:] = (0, 0, 0)

        avg_fps = sum(self.fps_history) / len(self.fps_history) if len(self.fps_history) > 0 else 0
        cv2.putText(stats_bg, f"FPS: {avg_fps:.1f}  |  Objects: {len(detections)}  |  Vision-Language AI",
                   (15, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        final_frame = np.vstack([stats_bg, final_frame])

        return final_frame

    def wrap_text(self, text, max_length):
        """Simple text wrapping"""
        words = text.split()
        lines = []
        current_line = []
        current_length = 0

        for word in words:
            if current_length + len(word) + 1 <= max_length:
                current_line.append(word)
                current_length += len(word) + 1
            else:
                if current_line:
                    lines.append(' '.join(current_line))
                current_line = [word]
                current_length = len(word)

        if current_line:
            lines.append(' '.join(current_line))

        return lines


def main(args=None):
    print("=" * 70)
    print("  🚀 Vision-Language-Action AI System")
    print("=" * 70)
    print("")

    # Set ROS_DOMAIN_ID if not set
    if 'ROS_DOMAIN_ID' not in os.environ:
        os.environ['ROS_DOMAIN_ID'] = '28'
        print("  ℹ️  Setting ROS_DOMAIN_ID=28")
        print("")

    print("  IMPORTANT: Make sure robot hardware is running!")
    print("")
    print("  If you see 'Camera not found' error:")
    print("    1. Open a new terminal")
    print("    2. Run: ./scripts/start_robot.sh")
    print("    3. Wait for robot to initialize")
    print("    4. Then run this script")
    print("")
    print("=" * 70)
    print("")

    # Initialize ROS2
    rclpy.init(args=args)

    # Create node
    try:
        node = VisionLLMNode()
    except Exception as e:
        print(f"❌ Failed to create node: {e}")
        rclpy.shutdown()
        sys.exit(1)

    # Launch image viewer
    print("🖼️  Launching AI vision viewer...")
    print("")
    print("=" * 70)
    print("  🤖 Complete AI Vision System Active")
    print("=" * 70)
    print("")
    print("  What You'll See:")
    print("")
    print("  VISUAL PERCEPTION:")
    print("    • YOLO11: Green boxes showing detected objects")
    print("    • DINOv2: Colored heatmap showing visual attention")
    print("")
    print("  AI REASONING (Bottom Panel):")
    print("    • Scene Description: What the AI understands")
    print("    • Suggested Action: What the robot should do")
    print("")
    print("  This is a complete vision-to-action AI pipeline!")
    print("  The LLM sees what YOLO+DINOv2 detect, then reasons")
    print("  about the scene and suggests intelligent actions.")
    print("")
    print("  Press Ctrl+C to stop")
    print("=" * 70)
    print("")

    viewer_process = None
    try:
        # Start rqt_image_view
        viewer_process = subprocess.Popen(
            ['ros2', 'run', 'rqt_image_view', 'rqt_image_view', '/vision_llm/output'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        # Give viewer time to start
        time.sleep(2)

        # Spin node
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        print("\n🛑 Shutting down AI system...")

        # Kill viewer process
        if viewer_process is not None:
            viewer_process.terminate()
            try:
                viewer_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                viewer_process.kill()

        # Shutdown ROS2
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

        print("")
        print("=" * 70)
        print("  ✅ Vision-Language-Action AI Stopped")
        print("=" * 70)
        print("")


if __name__ == '__main__':
    main()
