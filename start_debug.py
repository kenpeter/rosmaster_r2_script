#!/usr/bin/env python3
"""
Start Autonomous Driving System with Full Visualization AND Debug Logging
Combined script: start_autonomous_with_viz.py + debug_logger.py
"""

import subprocess
import time
import os
import signal
import sys
import threading
import json
import datetime

# Try to import rclpy for logging
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from geometry_msgs.msg import Twist
    RCLPY_AVAILABLE = True
except ImportError:
    RCLPY_AVAILABLE = False
    print("⚠️  rclpy not found. Logging features will be disabled.")
    print("   Please source your ROS2 environment: source install/setup.bash")

# Configuration
WORKSPACE_DIR = os.path.expanduser("~/yahboomcar_ros2_ws/yahboomcar_ws")
SETUP_CMD = "source /opt/ros/humble/setup.bash && source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash"
LOG_FILE = "autonomous_debug_log.txt"

# Logger Configuration will be initialized inside the class
LOG_TOPICS = []

class DebugLogger(Node):
    def __init__(self):
        super().__init__('debug_logger_integrated')

        # Initialize LOG_TOPICS with imported message types
        global LOG_TOPICS
        LOG_TOPICS = [
            ('/autonomous/detections', String, 'PERCEPTION'),
            ('/autonomous/llm/prompt', String, 'LLM_PROMPT'),
            ('/autonomous/llm/response', String, 'LLM_RESPONSE'),
            ('/autonomous/state', String, 'STATE'),
            ('/cmd_vel', Twist, 'ACTION')
        ]

        self.log_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), LOG_FILE)
        
        # Initialize log file
        with open(self.log_file, 'w') as f:
            f.write(f"=== AUTONOMOUS DEBUG LOG STARTED AT {datetime.datetime.now()} ===\n")
            f.write("Format: [TIMESTAMP] [COMPONENT] Data\n\n")
        
        self.get_logger().info(f"Logging to: {self.log_file}")
        
        # Discover and subscribe
        self.discover_topics()
        self.create_subscriptions()

    def discover_topics(self):
        """Run ros2 topic list to see what's actually available and auto-subscribe"""
        try:
            result = subprocess.run(['ros2', 'topic', 'list', '-t'], capture_output=True, text=True)
            if result.returncode == 0:
                self.log_to_file("SYSTEM", f"Available Topics:\n{result.stdout}")
                
                # Dynamic subscription based on keywords
                lines = result.stdout.strip().split('\n')
                keywords = ['prompt', 'response', 'llama', 'decision', 'dino', 'yolo', 'caption']
                
                for line in lines:
                    parts = line.split()
                    if len(parts) >= 2:
                        topic_name = parts[0]
                        msg_type_str = parts[1].strip('[]')
                        
                        # Check if we already subscribed
                        if any(t[0] == topic_name for t in LOG_TOPICS):
                            continue
                            
                        # Check for keywords
                        if any(k in topic_name.lower() for k in keywords):
                            self.get_logger().info(f"Auto-discovering interesting topic: {topic_name} [{msg_type_str}]")
                            self.add_dynamic_subscription(topic_name, msg_type_str)
        except Exception as e:
            self.log_to_file("SYSTEM", f"Error listing topics: {e}")

    def add_dynamic_subscription(self, topic_name, msg_type_str):
        """Dynamically import message type and subscribe"""
        try:
            parts = msg_type_str.replace('/', '.').split('.')
            if len(parts) >= 3:
                module_name = '.'.join(parts[:-1])
                class_name = parts[-1]
                module = __import__(module_name, fromlist=[class_name])
                msg_class = getattr(module, class_name)
                
                self.create_subscription(
                    msg_class,
                    topic_name,
                    lambda msg, l=f"AUTO_{topic_name}": self.log_callback(msg, l),
                    10
                )
        except Exception as e:
            self.get_logger().warn(f"Failed to dynamic subscribe to {topic_name}: {e}")

    def create_subscriptions(self):
        for topic, msg_type, label in LOG_TOPICS:
            self.create_subscription(
                msg_type,
                topic,
                lambda msg, l=label: self.log_callback(msg, l),
                10
            )

    def log_callback(self, msg, label):
        timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        content = ""
        if hasattr(msg, 'data'):
            content = msg.data
            if content.strip().startswith('{'):
                try:
                    parsed = json.loads(content)
                    content = json.dumps(parsed, indent=2) 
                except:
                    pass
        elif hasattr(msg, 'linear') and hasattr(msg, 'angular'):
            content = f"Linear: x={msg.linear.x:.2f}, Angular: z={msg.angular.z:.2f}"
        else:
            content = str(msg)

        entry = f"[{timestamp}] [{label}] {content}\n"
        self.log_to_file_raw(entry)

    def log_to_file(self, label, content):
        timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        entry = f"[{timestamp}] [{label}] {content}\n"
        self.log_to_file_raw(entry)

    def log_to_file_raw(self, entry):
        with open(self.log_file, 'a') as f:
            f.write(entry)

def run_terminal(title, command, geometry=None):
    """Launch a new gnome-terminal window"""
    full_cmd = f"gnome-terminal --title=\"{title}\""
    if geometry:
        full_cmd += f" --geometry={geometry}"
    bash_cmd = f"{SETUP_CMD} && {command}; exec bash"
    full_cmd += f" -- bash -c '{bash_cmd}'"
    print(f"🖥️  Opening {title}...")
    subprocess.Popen(full_cmd, shell=True)

def cleanup_processes():
    """Kill any existing autonomous and robot processes"""
    print("\n🧹 Cleaning up existing processes...")
    subprocess.run("pkill -f 'ros2 run autonomous_driving'", shell=True)
    subprocess.run("pkill -f 'ros2 launch autonomous_driving'", shell=True)
    subprocess.run("pkill -f 'ros2 launch yahboomcar_bringup'", shell=True)
    time.sleep(2)

def main():
    print("="*71)
    print("  🚀 Autonomous System + Visualization + Debug Logger")
    print("="*71)
    
    if os.path.exists(WORKSPACE_DIR):
        os.chdir(WORKSPACE_DIR)
    
    cleanup_processes()
    processes = []
    
    try:
        # 0. Launch Robot Hardware (Base, Lidar, Camera)
        print("🤖 Launching Robot Hardware (Base, Lidar, Camera)...")
        hardware_cmd = f"{SETUP_CMD} && source ~/yahboomcar_ros2_ws/software/library_ws/install/setup.bash && export ROS_DOMAIN_ID=28 && ros2 launch yahboomcar_bringup yahboomcar_bringup_R2_full_launch.py"
        # We run this in a separate terminal so user can see hardware status/errors
        run_terminal("Robot Hardware", hardware_cmd, geometry="100x20")
        time.sleep(10) # Wait for hardware to initialize

        # 1. Launch autonomous driving nodes
        print("📡 Launching autonomous driving nodes...")
        launch_cmd = f"{SETUP_CMD} && export ROS_DOMAIN_ID=28 && ros2 launch autonomous_driving autonomous_driving_launch.py enable_autonomous:=true"
        main_launch = subprocess.Popen(launch_cmd, shell=True, executable="/bin/bash")
        processes.append(main_launch)
        time.sleep(5)

        # 3. Launch Visualizations
        run_terminal("YOLO Detections", "sleep 3 && ros2 run rqt_image_view rqt_image_view /autonomous/debug_image")
        
        dino_cmd = """
        sleep 3
        if ros2 topic list | grep -q '/autonomous/dino_features'; then
            ros2 run rqt_image_view rqt_image_view /autonomous/dino_features
        else
            echo '⚠️  DINOv3 visualization topic not available yet'
            ros2 topic echo /autonomous/detections
        fi
        """
        run_terminal("DINOv3 Features", dino_cmd)
        
        llm_cmd = """
        echo 'Waiting for decisions...'
        ros2 topic echo /autonomous/decision
        """
        run_terminal("TinyLLM Reasoning", llm_cmd, geometry="120x30")
        
        run_terminal("3D SLAM World", "cd ~/yahboomcar_ros2_ws/yahboomcar_ws/scripts && sleep 3 && ./show_3d_world.py")

        print("")
        print("✅ System Launched.")
        print("📝 Debug logging active: ./scripts/autonomous_debug_log.txt")
        print("⚠️  Press Ctrl+C to stop everything.")
        print("")

        # 3. Start Logging (Blocking)
        if RCLPY_AVAILABLE:
            rclpy.init()
            logger = DebugLogger()
            try:
                rclpy.spin(logger)
            except KeyboardInterrupt:
                pass
            finally:
                logger.destroy_node()
                rclpy.shutdown()
        else:
            # Fallback wait if rclpy is missing
            main_launch.wait()

    except KeyboardInterrupt:
        print("\n🛑 Stop signal received.")
    finally:
        if 'main_launch' in locals() and main_launch.poll() is None:
            main_launch.terminate()
        cleanup_processes()
        print("✅ System stopped.")

if __name__ == "__main__":
    main()
