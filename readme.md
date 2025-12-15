ros2 topic list


systemctl --user status astra_cam


  ⎿  Camera is free

● Perfect! Camera is free now.

  Now launch JUST the camera to see the dense 3D world:

  export DISPLAY=:0
  export ROS_DOMAIN_ID=28
  source /opt/ros/humble/setup.bash
  source /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
  ros2 launch astra_camera astra.launch.py

  Run this command in your terminal. Once the camera starts, open RViz:

  # In a NEW terminal:
  export DISPLAY=:0
  export ROS_DOMAIN_ID=28
  source /opt/ros/humble/setup.bash
  rviz2

  In RViz:
  1. Fixed Frame: camera_link
  2. Add → PointCloud2
  3. Topic: /camera/depth/points
  4. Color Transformer: RGB8
  5. Size (m): 0.01

  This will show you 307,200 dense 3D colored points (640x480 resolution) - a REAL 3D world! Place objects 0.5-3 meters in front of the camera to see them in full 3D.




  killall -9 rtabmap rviz2 rgbd_odometry 2>/dev/null
  sleep 2

  ros2 launch show_slam.py



  source /opt/ros/humble/setup.bash
  source /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash


  ros2 launch /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/scripts/show_slam.py

