#!/usr/bin/env python3
"""
ONE SCRIPT TO SHOW 3D WORLD
============================
Run this to see your 3D colored world visualization!

Tries RGB-D camera first (REAL 3D), falls back to LiDAR+Camera fusion if needed.
"""

import os
import sys
import subprocess
import time
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

# Colors for terminal output
GREEN = '\033[0;32m'
YELLOW = '\033[1;33m'
BLUE = '\033[0;34m'
RED = '\033[0;31m'
BOLD = '\033[1m'
NC = '\033[0m'

WORKSPACE = Path("/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws")

def print_header():
    """Print welcome banner"""
    os.system('clear')
    print(f"{BOLD}{BLUE}")
    print("╔═══════════════════════════════════════════════════════════════╗")
    print("║                                                               ║")
    print("║         🌍  3D WORLD VISUALIZATION  🌍                        ║")
    print("║                                                               ║")
    print("║         Your ONE script for 3D colored world!                ║")
    print("║                                                               ║")
    print("╚═══════════════════════════════════════════════════════════════╝")
    print(f"{NC}\n")

def check_camera():
    """Check if Astra camera is available"""
    print(f"{YELLOW}🔍 Checking for RGB-D camera...{NC}")

    result = subprocess.run(
        "lsusb | grep -i 'orbbec\|astra\|primesense'",
        shell=True,
        capture_output=True,
        text=True
    )

    if result.returncode == 0:
        print(f"{GREEN}✓ RGB-D camera found: {result.stdout.strip()}{NC}")
        return True
    else:
        print(f"{YELLOW}⚠ No RGB-D camera detected{NC}")
        return False

def build_packages():
    """Build required packages"""
    print(f"\n{YELLOW}📦 Checking packages...{NC}")

    # Check if fusion package exists
    fusion_build = WORKSPACE / "build/lidar_camera_fusion"

    if not fusion_build.exists():
        print(f"{YELLOW}Building lidar_camera_fusion (first time)...{NC}")
        os.chdir(WORKSPACE)
        result = subprocess.run(
            "source /opt/ros/humble/setup.bash && colcon build --packages-select lidar_camera_fusion",
            shell=True,
            executable='/bin/bash'
        )
        if result.returncode != 0:
            print(f"{RED}✗ Build failed{NC}")
            return False

    print(f"{GREEN}✓ Packages ready{NC}")
    return True

def create_rviz_config(pointcloud_topic="/colored_pointcloud", fixed_frame="camera_link"):
    """Create RViz configuration for 3D visualization"""
    config = f"""
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Name: Grid
      Plane: XY
      Plane Cell Count: 20
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Class: rviz_default_plugins/PointCloud2
      Color: 255; 255; 255
      Color Transformer: RGB8
      Decay Time: 0
      Enabled: true
      Name: Colored 3D World
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 5
      Size (m): 0.02
      Style: Points
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: {pointcloud_topic}
      Use Fixed Frame: true
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Class: rviz_default_plugins/LaserScan
      Color: 255; 0; 0
      Decay Time: 0
      Enabled: true
      Name: LiDAR Scan
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 5
      Size (m): 0.05
      Style: Points
      Topic:
        Depth: 50
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /scan
      Use Fixed Frame: true
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: {fixed_frame}
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 3
      Enable Stereo Rendering:
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Size: 0.05
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 0.5
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 3.14
    Saved: ~
"""

    config_file = WORKSPACE / "scripts/3d_world.rviz"
    with open(config_file, 'w') as f:
        f.write(config)
    return str(config_file)

def launch_fusion_3d(rviz_config):
    """Launch complete robot + LiDAR + Camera fusion for colored 3D point cloud"""
    calib_file = WORKSPACE / "src/lidar_camera_fusion/config/default_calibration.yaml"

    launch_content = f"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Include the full robot system
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('yahboomcar_bringup'), 'launch'),
            '/yahboomcar_bringup_R2_full_launch.py'
        ])
    )

    # LaserScan to PointCloud2 converter
    laser_to_pc = Node(
        package='lidar_camera_fusion',
        executable='laser_to_pointcloud',
        name='laser_to_pointcloud',
        parameters=[{{
            'scan_topic': '/scan',
            'pointcloud_topic': '/lidar_pointcloud',
            'fixed_frame': 'laser_link',
            'height_offset': 0.15,
        }}],
        output='screen'
    )

    # LiDAR-Camera fusion for colored point cloud
    fusion = Node(
        package='lidar_camera_fusion',
        executable='lidar_camera_fusion_node',
        name='lidar_camera_fusion',
        parameters=[{{
            'pointcloud_topic': '/lidar_pointcloud',
            'image_topic': '/camera/color/image_raw',
            'output_topic': '/colored_pointcloud',
            'calibration_file': '{calib_file}',
            'camera_frame': 'camera_link',
            'lidar_frame': 'laser_link',
        }}],
        output='screen'
    )

    # RViz visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '{rviz_config}'],
        output='screen'
    )

    return LaunchDescription([
        robot_launch,
        laser_to_pc,
        fusion,
        rviz,
    ])
"""

    temp_launch = "/tmp/show_3d_world_fusion.launch.py"
    with open(temp_launch, 'w') as f:
        f.write(launch_content)

    return temp_launch

def launch_lidar_fusion(rviz_config):
    """Launch LiDAR + Camera fusion system"""
    calib_file = WORKSPACE / "src/lidar_camera_fusion/config/default_calibration.yaml"

    launch_content = f"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # LiDAR
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/install/my_ydlidar_ros2_driver/share/my_ydlidar_ros2_driver/launch/ydlidar_launch.py'
        )
    )

    # LaserScan to PointCloud2 converter
    laser_to_pc = Node(
        package='lidar_camera_fusion',
        executable='laser_to_pointcloud',
        name='laser_to_pointcloud',
        parameters=[{{
            'scan_topic': '/scan',
            'pointcloud_topic': '/lidar_pointcloud',
            'fixed_frame': 'laser',
            'height_offset': 0.15,
        }}],
        output='screen'
    )

    # LiDAR-Camera fusion for colorization
    fusion = Node(
        package='lidar_camera_fusion',
        executable='lidar_camera_fusion_node',
        name='lidar_camera_fusion',
        parameters=[{{
            'pointcloud_topic': '/lidar_pointcloud',
            'image_topic': '/camera/color/image_raw',
            'output_topic': '/colored_pointcloud',
            'calibration_file': '{calib_file}',
            'camera_frame': 'camera_link',
            'lidar_frame': 'laser',
        }}],
        output='screen'
    )

    # RViz visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '{rviz_config}'],
        output='screen'
    )

    return LaunchDescription([
        lidar_launch,
        laser_to_pc,
        fusion,
        rviz,
    ])
"""

    temp_launch = "/tmp/show_3d_world_lidar.launch.py"
    with open(temp_launch, 'w') as f:
        f.write(launch_content)

    return temp_launch

def main():
    print_header()

    # Build packages if needed
    if not build_packages():
        print(f"\n{RED}Failed to build packages. Exiting.{NC}")
        sys.exit(1)

    # Check for camera
    has_camera = check_camera()

    print(f"\n{GREEN}🎨 Launching LiDAR + Camera Fusion 3D Visualization{NC}")

    # Create RViz config for fusion
    print(f"\n{YELLOW}⚙️  Creating visualization config...{NC}")
    rviz_config = create_rviz_config(pointcloud_topic="/colored_pointcloud", fixed_frame="camera_link")
    print(f"{GREEN}✓ Config ready{NC}")

    # Create launch file
    print(f"\n{YELLOW}🚀 Preparing launch system...{NC}")
    launch_file = launch_fusion_3d(rviz_config)
    print(f"{GREEN}✓ Launch file ready: {launch_file}{NC}")

    # Show what will launch
    print(f"\n{BLUE}═══════════════════════════════════════════════════════════════{NC}")
    print(f"{BOLD}{GREEN}Starting 3D World Visualization - FULL SYSTEM{NC}")
    print(f"{BLUE}═══════════════════════════════════════════════════════════════{NC}\n")
    print("This will launch:")
    print(f"  {GREEN}✓{NC} R2 Robot Hardware (Ackermann steering)")
    print(f"  {GREEN}✓{NC} YDLiDAR (360° scanning)")
    print(f"  {GREEN}✓{NC} Astra Plus Camera (RGB + Depth)")
    print(f"  {GREEN}✓{NC} IMU + EKF Sensor Fusion")
    print(f"  {GREEN}✓{NC} LiDAR to PointCloud converter")
    print(f"  {GREEN}✓{NC} Camera-LiDAR Fusion (colored points)")
    print(f"  {GREEN}✓{NC} RViz 3D viewer")
    print(f"\n{BOLD}{YELLOW}What you'll see in RViz:{NC}")
    print(f"  🌍 {BOLD}Colored 3D Point Cloud{NC} - LiDAR points with camera colors")
    print(f"  📡 Red LiDAR scan - Raw 2D sensor data")
    print(f"  📏 Grid - Reference floor\n")

    print(f"{GREEN}🚀 Launching now...{NC}\n")
    time.sleep(2)

    # Launch the system
    os.chdir(WORKSPACE)
    cmd = f"source /opt/ros/humble/setup.bash && source /home/jetson/yahboomcar_ros2_ws/software/library_ws/install/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=28 && ros2 launch {launch_file}"

    try:
        subprocess.run(cmd, shell=True, executable='/bin/bash', check=True)
    except KeyboardInterrupt:
        print(f"\n{YELLOW}Shutting down...{NC}")
        sys.exit(0)
    except subprocess.CalledProcessError as e:
        print(f"\n{RED}✗ Launch failed: {e}{NC}")
        print(f"{RED}  Please check the output above for errors from ROS2 nodes.{NC}")
        sys.exit(1)


if __name__ == '__main__':
    main()