#!/usr/bin/env python3
"""
COMPLETE 3D SLAM SYSTEM LAUNCHER
=================================
This script launches all hardware drivers and RTAB-Map SLAM system.
Run this single script to build a 3D map of your environment!

Hardware launched:
- Yahboom R2 base drivers (motors, encoders)
- IMU filter (orientation estimation)
- YDLidar TG30 (3D LiDAR scanner)
- Astra camera (RGB-D vision)

SLAM launched:
- RTAB-Map visual odometry
- RTAB-Map SLAM (mapping + loop closure)
- RViz visualization

Author: Merged from show_slam.py and rtabmap_launch.py
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ========================================
    # PACKAGE DIRECTORIES
    # ========================================
    yahboomcar_bringup_dir = get_package_share_directory('yahboomcar_bringup')
    my_ydlidar_driver_dir = get_package_share_directory('my_ydlidar_ros2_driver')
    astra_camera_dir = get_package_share_directory('astra_camera')
    scripts_dir = os.path.dirname(os.path.realpath(__file__))

    # ========================================
    # CONFIGURATION FILES
    # ========================================
    imu_filter_config = os.path.join(yahboomcar_bringup_dir, 'param', 'imu_filter_param.yaml')
    ydlidar_config = os.path.join(my_ydlidar_driver_dir, 'params', 'ydlidar.yaml')
    rtabmap_params_file = os.path.join(scripts_dir, 'rtabmap_params.yaml')
    rviz_config_file = '/opt/ros/humble/share/rtabmap_launch/launch/config/rgbd.rviz'

    # Load RTAB-Map parameters from YAML
    with open(rtabmap_params_file, 'r') as f:
        rtabmap_params = yaml.safe_load(f)['/**']['ros__parameters']

    # ========================================
    # RTAB-MAP TOPIC REMAPPINGS
    # ========================================
    # Using IR (infrared) instead of RGB because color camera is not enabled
    # Scan disabled for camera-only SLAM test
    rtabmap_remappings = [
        ('rgb/image', '/camera/ir/image_raw'),
        ('rgb/camera_info', '/camera/ir/camera_info'),
        ('depth/image', '/camera/depth/image_raw'),
    ]

    # ========================================
    # LAUNCH DESCRIPTION
    # ========================================
    return LaunchDescription([
        # Set logging environment for better RTAB-Map debug output
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # ========================================
        # HARDWARE: YAHBOOM R2 BASE DRIVERS
        # ========================================
        LogInfo(msg="[1/7] Launching Yahboom R2 Base Drivers..."),
        Node(
            package='yahboomcar_bringup',
            executable='Ackman_driver_R2',
            name='ackman_driver',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        ),
        Node(
            package='yahboomcar_base_node',
            executable='base_node_R2',
            name='base_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        ),

        # ========================================
        # HARDWARE: IMU FILTER
        # ========================================
        LogInfo(msg="[2/7] Launching IMU Filter..."),
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            parameters=[imu_filter_config],
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        ),

        # ========================================
        # HARDWARE: YDLIDAR TG30 (DISABLED FOR CAMERA-ONLY TEST)
        # ========================================
        # LogInfo(msg="[3/7] Launching YDLidar TG30 Driver..."),
        # Node(
        #     package='my_ydlidar_ros2_driver',
        #     executable='my_ydlidar_ros2_driver_node',
        #     name='ydlidar_driver',
        #     output='screen',
        #     parameters=[ydlidar_config],
        #     arguments=['--ros-args', '--log-level', 'info']
        # ),
        #
        # # Transform: base_link -> laser_link
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='tf_base_to_laser',
        #     arguments=['0.0435', '0.000053', '0.11', '3.14', '0', '0', 'base_link', 'laser_link']
        # ),

        # ========================================
        # HARDWARE: ASTRA CAMERA
        # ========================================
        LogInfo(msg="[3/5] Launching Astra RGB-D Camera (Camera-Only SLAM Mode)..."),

        # Transform: base_link -> camera_link (mount position on robot)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_camera',
            arguments=['0.1', '0', '0.15', '0', '0', '0', 'base_link', 'camera_link']
        ),

        # Transform: camera_link -> camera_ir_optical_frame (optical frame convention)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_camera_to_ir_optical',
            arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link', 'camera_ir_optical_frame']
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(astra_camera_dir, 'launch', 'astra.launch.py')
            ),
            launch_arguments={'connection_delay': '2000'}.items()
        ),

        # ========================================
        # WAIT FOR SENSOR INITIALIZATION
        # ========================================
        LogInfo(msg="[4/5] Waiting 5 seconds for camera to stabilize..."),
        ExecuteProcess(cmd=['sleep', '5']),

        # ========================================
        # SLAM: RTAB-MAP VISUAL ODOMETRY
        # ========================================
        LogInfo(msg="[5/5] Launching RTAB-Map Visual Odometry (Camera-Only)..."),
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[rtabmap_params],
            remappings=rtabmap_remappings,
            arguments=['--ros-args', '--log-level', 'info']
        ),

        # ========================================
        # SLAM: RTAB-MAP MAIN SLAM NODE
        # ========================================
        LogInfo(msg="[5/5] Launching RTAB-Map SLAM System (Camera-Only)..."),
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[rtabmap_params],
            remappings=rtabmap_remappings,
            arguments=['--udebug', '--ros-args', '--log-level', 'info']
        ),

        # ========================================
        # VISUALIZATION: RVIZ2
        # ========================================
        LogInfo(msg="[5/5] Launching RViz2 Visualization..."),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),

        LogInfo(msg="========================================"),
        LogInfo(msg="RTAB-Map Camera-Only SLAM Ready!"),
        LogInfo(msg="Using IR + Depth camera (no LiDAR)."),
        LogInfo(msg="Drive the robot to start building the map."),
        LogInfo(msg="========================================"),
    ])
