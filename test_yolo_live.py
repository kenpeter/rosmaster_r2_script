#!/usr/bin/env python3
"""
YOLO11 Live Detection Viewer
Single script that runs YOLO detection and displays results
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from ultralytics import YOLO
import torch
import os
import subprocess
import signal
import sys

class YoloLiveNode(Node):
    def __init__(self):
        super().__init__('yolo_live_test')

        self.get_logger().info("=" * 70)
        self.get_logger().info("  📹 YOLO11 Live Detection Viewer")
        self.get_logger().info("=" * 70)

        # Load YOLO model
        model_path = os.path.expanduser("~/yahboomcar_ros2_ws/yahboomcar_ws/models/yolo11s.pt")
        self.get_logger().info("🤖 Loading YOLO11 model...")

        try:
            self.model = YOLO(model_path)
            self.get_logger().info("✅ YOLO11-Small loaded")

            if torch.cuda.is_available():
                self.get_logger().info(f"🚀 Using GPU: {torch.cuda.get_device_name(0)}")
            else:
                self.get_logger().info("⚠️  Using CPU (slower)")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load model: {e}")
            raise

        # Detection parameters
        self.conf_threshold = 0.5

        # CV Bridge
        self.bridge = CvBridge()

        # Publisher for annotated images
        self.image_pub = self.create_publisher(Image, '/yolo_live/detections', 10)

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
        self.get_logger().info("  🎬 Live Detection Started")
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
        """Run YOLO detection and publish annotated frame"""
        # Run YOLO detection
        start_time = time.time()
        results = self.model(frame, conf=self.conf_threshold, verbose=False)
        inference_time = (time.time() - start_time) * 1000  # ms

        # Calculate FPS
        fps = 1000 / inference_time if inference_time > 0 else 0
        self.fps_history.append(fps)
        if len(self.fps_history) > 30:
            self.fps_history.pop(0)
        avg_fps = sum(self.fps_history) / len(self.fps_history)

        # Draw detections
        annotated_frame = frame.copy()
        num_detections = 0

        if len(results) > 0 and len(results[0].boxes) > 0:
            num_detections = len(results[0].boxes)

            for box in results[0].boxes:
                # Get box coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                # Get class and confidence
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = self.model.names[cls]

                # Draw bounding box (thick green)
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 3)

                # Draw label with background
                label = f"{class_name} {conf:.2f}"
                label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)

                # Draw label background (green)
                cv2.rectangle(annotated_frame,
                            (x1, y1 - label_size[1] - 15),
                            (x1 + label_size[0] + 10, y1),
                            (0, 255, 0), -1)

                # Draw label text (black)
                cv2.putText(annotated_frame, label, (x1 + 5, y1 - 8),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)

        # Draw info overlay (top-left corner)
        overlay_y = 35
        line_height = 35

        # Background for text (black rectangle)
        cv2.rectangle(annotated_frame, (10, 10), (400, 180), (0, 0, 0), -1)

        # Draw stats
        cv2.putText(annotated_frame, f"FPS: {avg_fps:.1f}", (20, overlay_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        overlay_y += line_height

        cv2.putText(annotated_frame, f"Inference: {inference_time:.1f} ms", (20, overlay_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        overlay_y += line_height

        cv2.putText(annotated_frame, f"Detections: {num_detections}", (20, overlay_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        overlay_y += line_height

        cv2.putText(annotated_frame, f"Confidence: {self.conf_threshold:.1f}", (20, overlay_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

        # Publish annotated frame
        try:
            msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            self.image_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish image: {e}")

        # Log stats every 2 seconds
        current_time = time.time()
        if current_time - self.last_log_time > 2.0:
            self.get_logger().info(
                f"📊 {avg_fps:.1f} FPS | {inference_time:.1f}ms | {num_detections} objects"
            )
            self.last_log_time = current_time


def main(args=None):
    print("=" * 70)
    print("  🚀 YOLO11 Live Detection - Real Camera Test")
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
        node = YoloLiveNode()
    except Exception as e:
        print(f"❌ Failed to create node: {e}")
        rclpy.shutdown()
        sys.exit(1)

    # Launch image viewer in a separate process
    print("🖼️  Launching image viewer...")
    print("")
    print("=" * 70)
    print("  📹 Live Camera View with YOLO11 Detection")
    print("=" * 70)
    print("")
    print("  You'll see:")
    print("    • Real-time camera feed from robot")
    print("    • Green boxes around detected objects")
    print("    • Object labels (person, car, cup, etc.)")
    print("    • FPS and inference time")
    print("")
    print("  Try pointing camera at:")
    print("    • People, pets, cars")
    print("    • Common objects (cup, bottle, phone)")
    print("    • Furniture, plants, appliances")
    print("")
    print("  Press Ctrl+C to stop")
    print("=" * 70)
    print("")

    viewer_process = None
    try:
        # Start rqt_image_view
        viewer_process = subprocess.Popen(
            ['ros2', 'run', 'rqt_image_view', 'rqt_image_view', '/yolo_live/detections'],
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
        print("  ✅ YOLO Live Detection Stopped")
        print("=" * 70)
        print("")


if __name__ == '__main__':
    main()
