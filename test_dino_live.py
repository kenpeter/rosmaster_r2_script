#!/usr/bin/env python3
"""
DINOv2 Live Feature Visualization
Single script that runs DINOv2 and displays attention maps
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

class DinoLiveNode(Node):
    def __init__(self):
        super().__init__('dino_live_test')

        self.get_logger().info("=" * 70)
        self.get_logger().info("  🦖 DINOv2 Live Feature Viewer")
        self.get_logger().info("=" * 70)

        # Load DINOv2 model
        self.get_logger().info("🤖 Loading DINOv2 model...")

        try:
            from transformers import AutoImageProcessor, AutoModel

            model_name = 'facebook/dinov2-small'
            self.processor = AutoImageProcessor.from_pretrained(model_name)
            self.model = AutoModel.from_pretrained(model_name)

            # Move to GPU if available
            if torch.cuda.is_available():
                self.model = self.model.cuda()
                self.get_logger().info(f"🚀 Using GPU: {torch.cuda.get_device_name(0)}")
            else:
                self.get_logger().info("⚠️  Using CPU (slower)")

            self.model.eval()
            self.get_logger().info("✅ DINOv2-Small loaded")

        except Exception as e:
            self.get_logger().error(f"❌ Failed to load model: {e}")
            raise

        # CV Bridge
        self.bridge = CvBridge()

        # Publisher for visualizations
        self.image_pub = self.create_publisher(Image, '/dino_live/visualization', 10)

        # Try to subscribe to camera topic
        self.camera_available = False
        self.get_logger().info("📷 Looking for camera topic...")

        # Wait a moment for topics to be available
        time.sleep(1)

        # Check if camera topic exists
        topic_list = self.get_topic_names_and_types()
        camera_topics = ['/camera/color/image_raw', '/camera/image_raw', '/image_raw', '/RGBD/RGB/Image']

        self.get_logger().info(f"Available topics: {len(topic_list)}")

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

        # FPS tracking
        self.fps_history = []
        self.last_log_time = time.time()

        self.get_logger().info("")
        self.get_logger().info("=" * 70)
        self.get_logger().info("  🎬 DINOv2 Feature Extraction Started")
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
        """Run DINOv2 feature extraction and visualize"""
        start_time = time.time()

        # Prepare image for DINOv2
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Resize to 224x224 for DINOv2
        image_resized = cv2.resize(frame_rgb, (224, 224))

        # Process with DINOv2
        with torch.no_grad():
            inputs = self.processor(images=image_resized, return_tensors="pt")

            if torch.cuda.is_available():
                inputs = {k: v.cuda() for k, v in inputs.items()}

            outputs = self.model(**inputs)

            # Get last hidden state (features)
            features = outputs.last_hidden_state  # Shape: [1, num_patches, hidden_dim]

        inference_time = (time.time() - start_time) * 1000  # ms

        # Calculate FPS
        fps = 1000 / inference_time if inference_time > 0 else 0
        self.fps_history.append(fps)
        if len(self.fps_history) > 30:
            self.fps_history.pop(0)
        avg_fps = sum(self.fps_history) / len(self.fps_history)

        # Create visualization
        vis_frame = self.create_visualization(frame, features, avg_fps, inference_time)

        # Publish visualization
        try:
            msg = self.bridge.cv2_to_imgmsg(vis_frame, encoding='bgr8')
            self.image_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish image: {e}")

        # Log stats every 2 seconds
        current_time = time.time()
        if current_time - self.last_log_time > 2.0:
            self.get_logger().info(
                f"📊 {avg_fps:.1f} FPS | {inference_time:.1f}ms | Feature dim: {features.shape[-1]}"
            )
            self.last_log_time = current_time

    def create_visualization(self, frame, features, fps, inference_time):
        """Create visualization of DINOv2 features"""
        # Get feature statistics
        feature_mean = features.mean().item()
        feature_std = features.std().item()
        feature_max = features.max().item()
        feature_min = features.min().item()

        # Create attention map visualization
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

        # Apply colormap
        attention_colored = cv2.applyColorMap(attention_resized, cv2.COLORMAP_JET)

        # Blend with original frame
        alpha = 0.4
        blended = cv2.addWeighted(frame, 1 - alpha, attention_colored, alpha, 0)

        # Draw info overlay (top-left)
        overlay_y = 35
        line_height = 35

        # Background for text
        cv2.rectangle(blended, (10, 10), (450, 250), (0, 0, 0), -1)

        # Draw stats
        cv2.putText(blended, f"FPS: {fps:.1f}", (20, overlay_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        overlay_y += line_height

        cv2.putText(blended, f"Inference: {inference_time:.1f} ms", (20, overlay_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        overlay_y += line_height

        cv2.putText(blended, f"Feature Mean: {feature_mean:.3f}", (20, overlay_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        overlay_y += line_height

        cv2.putText(blended, f"Feature Std: {feature_std:.3f}", (20, overlay_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        overlay_y += line_height

        cv2.putText(blended, f"Range: [{feature_min:.2f}, {feature_max:.2f}]", (20, overlay_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # Add title
        cv2.putText(blended, "DINOv2 Attention Map", (20, h - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        return blended


def main(args=None):
    print("=" * 70)
    print("  🚀 DINOv2 Live Feature Visualization")
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
        node = DinoLiveNode()
    except Exception as e:
        print(f"❌ Failed to create node: {e}")
        rclpy.shutdown()
        sys.exit(1)

    # Launch image viewer in a separate process
    print("🖼️  Launching visualization viewer...")
    print("")
    print("=" * 70)
    print("  🦖 DINOv2 Feature Visualization")
    print("=" * 70)
    print("")
    print("  You'll see:")
    print("    • Real-time camera feed")
    print("    • Attention heatmap overlay (colored)")
    print("    • Red/yellow = high attention areas")
    print("    • Blue = low attention areas")
    print("    • FPS and feature statistics")
    print("")
    print("  DINOv2 shows what the AI 'pays attention to'")
    print("  in the scene - useful for understanding")
    print("  visual perception!")
    print("")
    print("  Press Ctrl+C to stop")
    print("=" * 70)
    print("")

    viewer_process = None
    try:
        # Start rqt_image_view
        viewer_process = subprocess.Popen(
            ['ros2', 'run', 'rqt_image_view', 'rqt_image_view', '/dino_live/visualization'],
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
        print("  ✅ DINOv2 Visualization Stopped")
        print("=" * 70)
        print("")


if __name__ == '__main__':
    main()
