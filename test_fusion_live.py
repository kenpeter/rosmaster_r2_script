#!/usr/bin/env python3
"""
YOLO11 + DINOv3 Fusion - Live AI Vision
Combines object detection (YOLO) with attention visualization (DINOv3)
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

class FusionVisionNode(Node):
    def __init__(self):
        super().__init__('fusion_vision_live')

        self.get_logger().info("=" * 70)
        self.get_logger().info("  🤖 YOLO11 + DINOv3 Fusion Vision")
        self.get_logger().info("=" * 70)
        self.get_logger().info("")

        # Load YOLO11 model
        self.get_logger().info("📦 Loading YOLO11 model...")
        model_path = os.path.expanduser("~/yahboomcar_ros2_ws/yahboomcar_ws/models/yolo11s.pt")

        try:
            from ultralytics import YOLO
            self.yolo_model = YOLO(model_path)
            self.get_logger().info("✅ YOLO11-Small loaded")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load YOLO11: {e}")
            raise

        # Load DINOv3 model
        self.get_logger().info("🦖 Loading DINOv3 model...")

        try:
            from transformers import AutoImageProcessor, AutoModel

            # Use DINOv3 if available, fallback to DINOv2
            try:
                model_name = 'facebook/dinov3-vit-small'
                self.dino_processor = AutoImageProcessor.from_pretrained(model_name)
                self.dino_model = AutoModel.from_pretrained(model_name)
                self.get_logger().info("✅ DINOv3-ViT-Small loaded")
            except:
                model_name = 'facebook/dinov2-small'
                self.dino_processor = AutoImageProcessor.from_pretrained(model_name)
                self.dino_model = AutoModel.from_pretrained(model_name)
                self.get_logger().info("✅ DINOv2-Small loaded (DINOv3 not available)")

            # Move to GPU if available
            if torch.cuda.is_available():
                self.dino_model = self.dino_model.cuda()
                self.get_logger().info(f"🚀 Using GPU: {torch.cuda.get_device_name(0)}")
            else:
                self.get_logger().info("⚠️  Using CPU (slower)")

            self.dino_model.eval()

        except Exception as e:
            self.get_logger().error(f"❌ Failed to load DINOv3: {e}")
            raise

        # Detection parameters
        self.conf_threshold = 0.5
        self.heatmap_alpha = 0.3  # Transparency of heatmap

        # CV Bridge
        self.bridge = CvBridge()

        # Publisher for fused visualization
        self.image_pub = self.create_publisher(Image, '/fusion_vision/output', 10)

        # Try to subscribe to camera topic
        self.camera_available = False
        self.get_logger().info("📷 Looking for camera topic...")

        # Wait a moment for topics to be available
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
        self.fps_history = []
        self.yolo_time_history = []
        self.dino_time_history = []
        self.last_log_time = time.time()

        self.get_logger().info("")
        self.get_logger().info("=" * 70)
        self.get_logger().info("  🎬 Fusion Vision Started")
        self.get_logger().info("=" * 70)
        self.get_logger().info("")

    def camera_callback(self, msg):
        """Process real camera frame"""
        try:
            # Convert ROS Image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_frame(frame)
        except Exception as e:
            self.get_logger().error(f"Error processing camera frame: {e}")

    def process_frame(self, frame):
        """Run both YOLO and DINOv3, then fuse visualizations"""
        total_start_time = time.time()

        # 1. Run DINOv3 for attention heatmap
        dino_start = time.time()
        attention_map = self.get_dino_attention(frame)
        dino_time = (time.time() - dino_start) * 1000  # ms

        # 2. Run YOLO for object detection
        yolo_start = time.time()
        yolo_results = self.yolo_model(frame, conf=self.conf_threshold, verbose=False)
        yolo_time = (time.time() - yolo_start) * 1000  # ms

        # 3. Create fused visualization
        fused_frame = self.create_fusion_visualization(frame, attention_map, yolo_results)

        # Calculate total FPS
        total_time = (time.time() - total_start_time) * 1000  # ms
        fps = 1000 / total_time if total_time > 0 else 0

        # Track performance
        self.fps_history.append(fps)
        self.yolo_time_history.append(yolo_time)
        self.dino_time_history.append(dino_time)

        if len(self.fps_history) > 30:
            self.fps_history.pop(0)
            self.yolo_time_history.pop(0)
            self.dino_time_history.pop(0)

        avg_fps = sum(self.fps_history) / len(self.fps_history)
        avg_yolo = sum(self.yolo_time_history) / len(self.yolo_time_history)
        avg_dino = sum(self.dino_time_history) / len(self.dino_time_history)

        # Add performance stats to visualization
        final_frame = self.add_stats_overlay(fused_frame, avg_fps, avg_yolo, avg_dino,
                                             len(yolo_results[0].boxes) if len(yolo_results) > 0 else 0)

        # Publish fused visualization
        try:
            msg = self.bridge.cv2_to_imgmsg(final_frame, encoding='bgr8')
            self.image_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish image: {e}")

        # Log stats every 2 seconds
        current_time = time.time()
        if current_time - self.last_log_time > 2.0:
            num_objects = len(yolo_results[0].boxes) if len(yolo_results) > 0 else 0
            self.get_logger().info(
                f"📊 {avg_fps:.1f} FPS | YOLO: {avg_yolo:.1f}ms | DINO: {avg_dino:.1f}ms | Objects: {num_objects}"
            )
            self.last_log_time = current_time

    def get_dino_attention(self, frame):
        """Extract attention map from DINOv3"""
        # Prepare image for DINOv3
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image_resized = cv2.resize(frame_rgb, (224, 224))

        # Process with DINOv3
        with torch.no_grad():
            inputs = self.dino_processor(images=image_resized, return_tensors="pt")

            if torch.cuda.is_available():
                inputs = {k: v.cuda() for k, v in inputs.items()}

            outputs = self.dino_model(**inputs)

            # Get last hidden state (features)
            features = outputs.last_hidden_state  # Shape: [1, num_patches, hidden_dim]

        # Create attention map
        # Average across feature dimension to get spatial attention
        attention = features[0, 1:, :].mean(dim=-1)  # Skip CLS token
        num_patches = int(np.sqrt(attention.shape[0]))

        # Reshape to 2D grid
        attention_map = attention.reshape(num_patches, num_patches)
        attention_map = attention_map.cpu().numpy()

        # Normalize to 0-255
        attention_map = (attention_map - attention_map.min()) / (attention_map.max() - attention_map.min() + 1e-8)
        attention_map = (attention_map * 255).astype(np.uint8)

        # Resize to match original frame
        h, w = frame.shape[:2]
        attention_resized = cv2.resize(attention_map, (w, h), interpolation=cv2.INTER_CUBIC)

        return attention_resized

    def create_fusion_visualization(self, frame, attention_map, yolo_results):
        """Fuse DINOv3 heatmap with YOLO detections"""
        # 1. Apply colormap to attention (heatmap)
        attention_colored = cv2.applyColorMap(attention_map, cv2.COLORMAP_JET)

        # 2. Blend attention heatmap with original frame
        blended = cv2.addWeighted(frame, 1 - self.heatmap_alpha, attention_colored, self.heatmap_alpha, 0)

        # 3. Draw YOLO detections on top of blended frame
        if len(yolo_results) > 0 and len(yolo_results[0].boxes) > 0:
            for box in yolo_results[0].boxes:
                # Get box coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                # Get class and confidence
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = self.yolo_model.names[cls]

                # Draw bounding box (bright green, thick)
                cv2.rectangle(blended, (x1, y1), (x2, y2), (0, 255, 0), 3)

                # Draw label with background
                label = f"{class_name} {conf:.2f}"
                label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)

                # Draw label background (green)
                cv2.rectangle(blended,
                            (x1, y1 - label_size[1] - 15),
                            (x1 + label_size[0] + 10, y1),
                            (0, 255, 0), -1)

                # Draw label text (black)
                cv2.putText(blended, label, (x1 + 5, y1 - 8),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)

        return blended

    def add_stats_overlay(self, frame, fps, yolo_time, dino_time, num_detections):
        """Add performance statistics overlay"""
        overlay_y = 35
        line_height = 35

        # Background for text (black rectangle)
        cv2.rectangle(frame, (10, 10), (450, 220), (0, 0, 0), -1)

        # Draw stats
        cv2.putText(frame, f"FPS: {fps:.1f}", (20, overlay_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        overlay_y += line_height

        cv2.putText(frame, f"YOLO: {yolo_time:.1f} ms", (20, overlay_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        overlay_y += line_height

        cv2.putText(frame, f"DINO: {dino_time:.1f} ms", (20, overlay_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        overlay_y += line_height

        cv2.putText(frame, f"Objects: {num_detections}", (20, overlay_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        overlay_y += line_height

        # Legend
        cv2.putText(frame, "Heatmap = DINOv3 Attention", (20, overlay_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 2)

        # Add title at bottom
        h, w = frame.shape[:2]
        cv2.putText(frame, "YOLO11 + DINOv3 Fusion Vision", (20, h - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        return frame


def main(args=None):
    print("=" * 70)
    print("  🚀 YOLO11 + DINOv3 Fusion Vision")
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
        node = FusionVisionNode()
    except Exception as e:
        print(f"❌ Failed to create node: {e}")
        rclpy.shutdown()
        sys.exit(1)

    # Launch image viewer in a separate process
    print("🖼️  Launching fusion visualization viewer...")
    print("")
    print("=" * 70)
    print("  🎨 Fusion Vision - What You'll See")
    print("=" * 70)
    print("")
    print("  YOLO11 (Object Detection):")
    print("    • Green boxes around detected objects")
    print("    • Labels showing what each object is")
    print("    • Confidence scores")
    print("")
    print("  DINOv3 (AI Attention):")
    print("    • Colored heatmap overlay (background)")
    print("    • Red/Yellow = high attention areas")
    print("    • Blue = low attention areas")
    print("    • Shows what the AI 'sees' as important")
    print("")
    print("  Watch how DINOv3's attention aligns with YOLO's")
    print("  detected objects - fascinating AI correlation!")
    print("")
    print("  Press Ctrl+C to stop")
    print("=" * 70)
    print("")

    viewer_process = None
    try:
        # Start rqt_image_view
        viewer_process = subprocess.Popen(
            ['ros2', 'run', 'rqt_image_view', 'rqt_image_view', '/fusion_vision/output'],
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
        print("\n🛑 Shutting down...")

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
        print("  ✅ Fusion Vision Stopped")
        print("=" * 70)
        print("")


if __name__ == '__main__':
    main()
