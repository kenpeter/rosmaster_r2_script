
   Access the UI at:
   • http://localhost:5000
   • http://127.0.1.1:5000

======================================================================
🚗 Verifying Autonomous Motor Control...
======================================================================
✅ Motor control enabled: /vel_raw topic active
✅ Control node: Publishing velocity commands
✅ Safety timeout: 1.0 seconds (stops if no commands)
✅ Max linear velocity: 0.5 m/s
✅ Max angular velocity: 1.0 rad/s

💡 TIP: Monitor motor commands with:
   ros2 topic echo /vel_raw

💡 TIP: Monitor autonomous decisions with:
   ros2 topic echo /autonomous/decision

🛑 EMERGENCY STOP: Press Ctrl+C or close this terminal


======================================================================
✅ ALL SYSTEMS OPERATIONAL
======================================================================

🤖 AUTONOMOUS DRIVING STATUS:
  ✅ Perception: YOLO11 + DINOv2 running
  ✅ Decision: Qwen3 LLM making decisions
  ✅ Control: MOTORS ENABLED - ROBOT WILL MOVE
  ✅ Tesla UI: http://localhost:5000

🚗 ROBOT BEHAVIOR:
  • Detects objects with camera + YOLO
  • Understands scene with DINOv2
  • Makes decisions with Qwen3 LLM
  • Sends commands to motors automatically
  • Stops if: obstacle detected, no path, or safety timeout

RTAB-Map 3D SLAM:
  • 3D point cloud mapping
  • Robot trajectory tracking
  • Loop closure detection

💡 MONITORING COMMANDS:
  • Motor commands: ros2 topic echo /vel_raw
  • LLM decisions: ros2 topic echo /autonomous/decision
  • Detections: ros2 topic echo /autonomous/detections

🛑 EMERGENCY STOP: Press Ctrl+C

[llm_decision_node-3] [INFO] [1766993690.612065117] [llm_decision_node]: 
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 🤖 AUTONOMOUS DRIVING DECISION PIPELINE (SPATIAL REASONING)
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 
[llm_decision_node-3] 🗺️  STEP 1: SPATIAL AWARENESS (2D GRID)
[llm_decision_node-3]    Local Map 360° (R=Robot, #=Obstacle, .=Clear):
[llm_decision_node-3]       #.....#######...
[llm_decision_node-3]       ......####.##...
[llm_decision_node-3]       ..#...#.#......#
[llm_decision_node-3]       .#......#.......
[llm_decision_node-3]       ...#....#......#
[llm_decision_node-3]       ................
[llm_decision_node-3]       ................
[llm_decision_node-3]       ..........#..#..
[llm_decision_node-3]       ........R.###.##
[llm_decision_node-3]       .............##.
[llm_decision_node-3]       ...........###..
[llm_decision_node-3]       ###.........#...
[llm_decision_node-3]       ..##...#....#...
[llm_decision_node-3]       ..........##....
[llm_decision_node-3]       #.#.............
[llm_decision_node-3]       #.####.#.....###
[llm_decision_node-3]    
[llm_decision_node-3] 
[llm_decision_node-3] 📊 STEP 2: SENSOR FUSION
[llm_decision_node-3]    • LIDAR: blocked (0.16m)
[llm_decision_node-3]    • Depth: no_data
[llm_decision_node-3]    • Vision: No objects
[llm_decision_node-3]    • Scene: Unknown (DINOv2 Disabled)
[llm_decision_node-3] 
[llm_decision_node-3] 🧠 STEP 3: QWEN3 SPATIAL REASONING
[llm_decision_node-3]    • Task: "Analyze map and sensors. Output JSON."
[llm_decision_node-3]    • Context Hints: ['Status: Moving (slow)']
[llm_decision_node-3]    • Raw Response: "{"action": "GO"}"
[llm_decision_node-3]    • Inference Time: 1004ms
[llm_decision_node-3] 
[llm_decision_node-3] 🎯 STEP 4: FINAL ROBOT ACTION
[llm_decision_node-3]    • Action: STOP
[llm_decision_node-3]    • Linear Velocity: 0.0 m/s
[llm_decision_node-3]    • Angular Velocity: 0.0 rad/s
[llm_decision_node-3]    • Control Status: ENABLED
[llm_decision_node-3] 
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 
[llm_decision_node-3] [INFO] [1766993690.616783684] [llm_decision_node]: 🔧 Averaging 5 scans (metadata matches)
[llm_decision_node-3] [INFO] [1766993690.620014603] [llm_decision_node]: 🔧 Scan quality: 360 total | -178 valid (-49.4%) | 269 inf | 0 nan | 0 too_close (<0.15m) | 269 too_far (>50.0m)
[llm_decision_node-3] [INFO] [1766993690.626877314] [llm_decision_node]: 🔧 Grid: 16x16 (256 cells) | 52 obstacles (20.3% filled) | 91 valid ranges processed
[llm_decision_node-3] [INFO] [1766993691.670001188] [llm_decision_node]: 
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 🤖 AUTONOMOUS DRIVING DECISION PIPELINE (SPATIAL REASONING)
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 
[llm_decision_node-3] 🗺️  STEP 1: SPATIAL AWARENESS (2D GRID)
[llm_decision_node-3]    Local Map 360° (R=Robot, #=Obstacle, .=Clear):
[llm_decision_node-3]       .....#######....
[llm_decision_node-3]       ..#.....##..#...
[llm_decision_node-3]       .......##.......
[llm_decision_node-3]       ........#.......
[llm_decision_node-3]       ...#....#...#..#
[llm_decision_node-3]       .....#..........
[llm_decision_node-3]       ................
[llm_decision_node-3]       ...........#..#.
[llm_decision_node-3]       ........R.###.#.
[llm_decision_node-3]       .............##.
[llm_decision_node-3]       ............##.#
[llm_decision_node-3]       #..........####.
[llm_decision_node-3]       ####.........#..
[llm_decision_node-3]       ...........#....
[llm_decision_node-3]       #....#.....#.#..
[llm_decision_node-3]       ..####.#......##
[llm_decision_node-3]    
[llm_decision_node-3] 
[llm_decision_node-3] 📊 STEP 2: SENSOR FUSION
[llm_decision_node-3]    • LIDAR: caution (0.59m)
[llm_decision_node-3]    • Depth: no_data
[llm_decision_node-3]    • Vision: No objects
[llm_decision_node-3]    • Scene: Unknown (DINOv2 Disabled)
[llm_decision_node-3] 
[llm_decision_node-3] 🧠 STEP 3: QWEN3 SPATIAL REASONING
[llm_decision_node-3]    • Task: "Analyze map and sensors. Output JSON."
[llm_decision_node-3]    • Context Hints: ['Status: Moving (slow)']
[llm_decision_node-3]    • Raw Response: "{"action": "GO"}"
[llm_decision_node-3]    • Inference Time: 1040ms
[llm_decision_node-3] 
[llm_decision_node-3] 🎯 STEP 4: FINAL ROBOT ACTION
[llm_decision_node-3]    • Action: SLOW
[llm_decision_node-3]    • Linear Velocity: 0.2 m/s
[llm_decision_node-3]    • Angular Velocity: 0.0 rad/s
[llm_decision_node-3]    • Control Status: ENABLED
[llm_decision_node-3] 
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 
[llm_decision_node-3] [INFO] [1766993691.674895695] [llm_decision_node]: 🔧 Averaging 5 scans (metadata matches)
[llm_decision_node-3] [INFO] [1766993691.676046761] [llm_decision_node]: 🔧 Scan quality: 360 total | -104 valid (-28.9%) | 232 inf | 0 nan | 0 too_close (<0.15m) | 232 too_far (>50.0m)
[llm_decision_node-3] [INFO] [1766993691.678466142] [llm_decision_node]: 🔧 Grid: 16x16 (256 cells) | 63 obstacles (24.6% filled) | 128 valid ranges processed
[llm_decision_node-3] [INFO] [1766993692.718279862] [llm_decision_node]: 
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 🤖 AUTONOMOUS DRIVING DECISION PIPELINE (SPATIAL REASONING)
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 
[llm_decision_node-3] 🗺️  STEP 1: SPATIAL AWARENESS (2D GRID)
[llm_decision_node-3]    Local Map 360° (R=Robot, #=Obstacle, .=Clear):
[llm_decision_node-3]       ....####.####...
[llm_decision_node-3]       ........##.##...
[llm_decision_node-3]       ........#.......
[llm_decision_node-3]       .......#...#....
[llm_decision_node-3]       ....#...###.....
[llm_decision_node-3]       ........##.#.###
[llm_decision_node-3]       .......####..##.
[llm_decision_node-3]       ........#...#.##
[llm_decision_node-3]       ........R.#..#..
[llm_decision_node-3]       .........##....#
[llm_decision_node-3]       ............##.#
[llm_decision_node-3]       .###........#.##
[llm_decision_node-3]       ##...........###
[llm_decision_node-3]       ...........##...
[llm_decision_node-3]       ..#........####.
[llm_decision_node-3]       ..#.#...........
[llm_decision_node-3]    
[llm_decision_node-3] 
[llm_decision_node-3] 📊 STEP 2: SENSOR FUSION
[llm_decision_node-3]    • LIDAR: blocked (0.16m)
[llm_decision_node-3]    • Depth: no_data
[llm_decision_node-3]    • Vision: No objects
[llm_decision_node-3]    • Scene: Unknown (DINOv2 Disabled)
[llm_decision_node-3] 
[llm_decision_node-3] 🧠 STEP 3: QWEN3 SPATIAL REASONING
[llm_decision_node-3]    • Task: "Analyze map and sensors. Output JSON."
[llm_decision_node-3]    • Context Hints: ['Status: Moving (slow)']
[llm_decision_node-3]    • Raw Response: "{"action": "GO"}"
[llm_decision_node-3]    • Inference Time: 1037ms
[llm_decision_node-3] 
[llm_decision_node-3] 🎯 STEP 4: FINAL ROBOT ACTION
[llm_decision_node-3]    • Action: STOP
[llm_decision_node-3]    • Linear Velocity: 0.0 m/s
[llm_decision_node-3]    • Angular Velocity: 0.0 rad/s
[llm_decision_node-3]    • Control Status: ENABLED
[llm_decision_node-3] 
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 
[llm_decision_node-3] [INFO] [1766993692.726683535] [llm_decision_node]: 🔧 Averaging 5 scans (metadata matches)
[llm_decision_node-3] [INFO] [1766993692.729020898] [llm_decision_node]: 🔧 Scan quality: 360 total | -183 valid (-50.8%) | 264 inf | 0 nan | 15 too_close (<0.15m) | 264 too_far (>50.0m)
[llm_decision_node-3] [INFO] [1766993692.733894029] [llm_decision_node]: 🔧 Grid: 16x16 (256 cells) | 27 obstacles (10.5% filled) | 81 valid ranges processed
[llm_decision_node-3] [INFO] [1766993692.737670176] [llm_decision_node]: 📊 Depth stats: 0/73728 valid pixels, 73728 zeros, range: 0-0mm
[perception_node-1] [INFO] [1766993693.397422247] [perception_node]: Using device: cuda
[perception_node-1] [INFO] [1766993693.398252953] [perception_node]: Loading YOLO model: /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/models/yolo11s.pt
[llm_decision_node-3] [INFO] [1766993693.819063847] [llm_decision_node]: 
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 🤖 AUTONOMOUS DRIVING DECISION PIPELINE (SPATIAL REASONING)
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 
[llm_decision_node-3] 🗺️  STEP 1: SPATIAL AWARENESS (2D GRID)
[llm_decision_node-3]    Local Map 360° (R=Robot, #=Obstacle, .=Clear):
[llm_decision_node-3]       ..........##.#..
[llm_decision_node-3]       ...........#....
[llm_decision_node-3]       ................
[llm_decision_node-3]       .............#..
[llm_decision_node-3]       ...............#
[llm_decision_node-3]       .........#....#.
[llm_decision_node-3]       ........#.#....#
[llm_decision_node-3]       ........##......
[llm_decision_node-3]       ........R.#.....
[llm_decision_node-3]       .........##.....
[llm_decision_node-3]       ................
[llm_decision_node-3]       ................
[llm_decision_node-3]       ###...........#.
[llm_decision_node-3]       ..............#.
[llm_decision_node-3]       ..............##
[llm_decision_node-3]       .#..........##.#
[llm_decision_node-3]    
[llm_decision_node-3] 
[llm_decision_node-3] 📊 STEP 2: SENSOR FUSION
[llm_decision_node-3]    • LIDAR: caution (1.57m)
[llm_decision_node-3]    • Depth: no_data
[llm_decision_node-3]    • Vision: No objects
[llm_decision_node-3]    • Scene: Unknown (DINOv2 Disabled)
[llm_decision_node-3] 
[llm_decision_node-3] 🧠 STEP 3: QWEN3 SPATIAL REASONING
[llm_decision_node-3]    • Task: "Analyze map and sensors. Output JSON."
[llm_decision_node-3]    • Context Hints: ['Status: Moving (slow)']
[llm_decision_node-3]    • Raw Response: "{"action": "GO"}"
[llm_decision_node-3]    • Inference Time: 1078ms
[llm_decision_node-3] 
[llm_decision_node-3] 🎯 STEP 4: FINAL ROBOT ACTION
[llm_decision_node-3]    • Action: SLOW
[llm_decision_node-3]    • Linear Velocity: 0.2 m/s
[llm_decision_node-3]    • Angular Velocity: 0.0 rad/s
[llm_decision_node-3]    • Control Status: ENABLED
[llm_decision_node-3] 
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 
[llm_decision_node-3] [INFO] [1766993693.824412124] [llm_decision_node]: 🔧 Averaging 5 scans (metadata matches)
[llm_decision_node-3] [INFO] [1766993693.826552811] [llm_decision_node]: 🔧 Scan quality: 360 total | -120 valid (-33.3%) | 240 inf | 0 nan | 0 too_close (<0.15m) | 240 too_far (>50.0m)
[llm_decision_node-3] [INFO] [1766993693.834535130] [llm_decision_node]: 🔧 Grid: 16x16 (256 cells) | 59 obstacles (23.0% filled) | 120 valid ranges processed
[perception_node-1] [INFO] [1766993694.088206934] [perception_node]: ✅ YOLO model loaded
[perception_node-1] [INFO] [1766993694.089045704] [perception_node]: DINOv2 disabled by parameter
[perception_node-1] [INFO] [1766993694.117234547] [perception_node]: 🚀 Perception node started
[perception_node-1] [INFO] [1766993694.118043524] [perception_node]:    Subscribing to: /camera/color/image_raw
[perception_node-1] [INFO] [1766993694.118737620] [perception_node]:    YOLO model: /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/models/yolo11s.pt
[perception_node-1] [INFO] [1766993694.119547621] [perception_node]:    DINOv2: disabled
[perception_node-1] [INFO] [1766993694.120359639] [perception_node]:    Processing every 10 frames
[llm_decision_node-3] [INFO] [1766993694.915711419] [llm_decision_node]: 
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 🤖 AUTONOMOUS DRIVING DECISION PIPELINE (SPATIAL REASONING)
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 
[llm_decision_node-3] 🗺️  STEP 1: SPATIAL AWARENESS (2D GRID)
[llm_decision_node-3]    Local Map 360° (R=Robot, #=Obstacle, .=Clear):
[llm_decision_node-3]       .............###
[llm_decision_node-3]       .........#####.#
[llm_decision_node-3]       .........###....
[llm_decision_node-3]       .........###.#..
[llm_decision_node-3]       ........###..#..
[llm_decision_node-3]       ...........#.#.#
[llm_decision_node-3]       ..........###.##
[llm_decision_node-3]       ..........###.#.
[llm_decision_node-3]       .......#R...#..#
[llm_decision_node-3]       ................
[llm_decision_node-3]       ...##.#.........
[llm_decision_node-3]       .#....#.##..##.#
[llm_decision_node-3]       #.........#.###.
[llm_decision_node-3]       ..#........###..
[llm_decision_node-3]       .........#......
[llm_decision_node-3]       #.#.......##....
[llm_decision_node-3]    
[llm_decision_node-3] 
[llm_decision_node-3] 📊 STEP 2: SENSOR FUSION
[llm_decision_node-3]    • LIDAR: blocked (0.16m)
[llm_decision_node-3]    • Depth: no_data
[llm_decision_node-3]    • Vision: No objects
[llm_decision_node-3]    • Scene: Unknown (DINOv2 Disabled)
[llm_decision_node-3] 
[llm_decision_node-3] 🧠 STEP 3: QWEN3 SPATIAL REASONING
[llm_decision_node-3]    • Task: "Analyze map and sensors. Output JSON."
[llm_decision_node-3]    • Context Hints: ['Status: Moving (slow)']
[llm_decision_node-3]    • Raw Response: "{"action": "GO"}"
[llm_decision_node-3]    • Inference Time: 1078ms
[llm_decision_node-3] 
[llm_decision_node-3] 🎯 STEP 4: FINAL ROBOT ACTION
[llm_decision_node-3]    • Action: STOP
[llm_decision_node-3]    • Linear Velocity: 0.0 m/s
[llm_decision_node-3]    • Angular Velocity: 0.0 rad/s
[llm_decision_node-3]    • Control Status: ENABLED
[llm_decision_node-3] 
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 
[llm_decision_node-3] [INFO] [1766993694.925097577] [llm_decision_node]: 🔧 Averaging 5 scans (metadata matches)
[llm_decision_node-3] [INFO] [1766993694.927294841] [llm_decision_node]: 🔧 Scan quality: 360 total | -108 valid (-30.0%) | 234 inf | 0 nan | 0 too_close (<0.15m) | 234 too_far (>50.0m)
[llm_decision_node-3] [INFO] [1766993694.931387443] [llm_decision_node]: 🔧 Grid: 16x16 (256 cells) | 57 obstacles (22.3% filled) | 126 valid ranges processed
[llm_decision_node-3] [INFO] [1766993696.207077756] [llm_decision_node]: 
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 🤖 AUTONOMOUS DRIVING DECISION PIPELINE (SPATIAL REASONING)
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 
[llm_decision_node-3] 🗺️  STEP 1: SPATIAL AWARENESS (2D GRID)
[llm_decision_node-3]    Local Map 360° (R=Robot, #=Obstacle, .=Clear):
[llm_decision_node-3]       ...........#####
[llm_decision_node-3]       .........##.#...
[llm_decision_node-3]       .........###....
[llm_decision_node-3]       ..........##.#..
[llm_decision_node-3]       ........###..#..
[llm_decision_node-3]       ...........#.###
[llm_decision_node-3]       ..........###..#
[llm_decision_node-3]       ..........###.#.
[llm_decision_node-3]       ........R...###.
[llm_decision_node-3]       ................
[llm_decision_node-3]       ......#....#...#
[llm_decision_node-3]       ............##.#
[llm_decision_node-3]       .##..#...##.##.#
[llm_decision_node-3]       .........#.##.##
[llm_decision_node-3]       #..#........#...
[llm_decision_node-3]       ##..............
[llm_decision_node-3]    
[llm_decision_node-3] 
[llm_decision_node-3] 📊 STEP 2: SENSOR FUSION
[llm_decision_node-3]    • LIDAR: caution (0.98m)
[llm_decision_node-3]    • Depth: no_data
[llm_decision_node-3]    • Vision: No objects
[llm_decision_node-3]    • Scene: Unknown (DINOv2 Disabled)
[llm_decision_node-3] 
[llm_decision_node-3] 🧠 STEP 3: QWEN3 SPATIAL REASONING
[llm_decision_node-3]    • Task: "Analyze map and sensors. Output JSON."
[llm_decision_node-3]    • Context Hints: ['Status: Moving (slow)']
[llm_decision_node-3]    • Raw Response: "{"action": "GO"}"
[llm_decision_node-3]    • Inference Time: 1260ms
[llm_decision_node-3] 
[llm_decision_node-3] 🎯 STEP 4: FINAL ROBOT ACTION
[llm_decision_node-3]    • Action: SLOW
[llm_decision_node-3]    • Linear Velocity: 0.2 m/s
[llm_decision_node-3]    • Angular Velocity: 0.0 rad/s
[llm_decision_node-3]    • Control Status: ENABLED
[llm_decision_node-3] 
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 
[llm_decision_node-3] [INFO] [1766993696.217275836] [llm_decision_node]: 🔧 Averaging 5 scans (metadata matches)
[llm_decision_node-3] [INFO] [1766993696.220259837] [llm_decision_node]: 🔧 Scan quality: 360 total | -96 valid (-26.7%) | 228 inf | 0 nan | 0 too_close (<0.15m) | 228 too_far (>50.0m)
[llm_decision_node-3] [INFO] [1766993696.228366735] [llm_decision_node]: 🔧 Grid: 16x16 (256 cells) | 63 obstacles (24.6% filled) | 132 valid ranges processed
[perception_node-1] [INFO] [1766993697.143006569] [perception_node]: Frame 10: 2 objects | Total: 2695.1ms (YOLO: 2693.0ms, DINO: 0.0ms, Fusion: 0.0ms)
[llm_decision_node-3] [INFO] [1766993697.540226063] [llm_decision_node]: 
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 🤖 AUTONOMOUS DRIVING DECISION PIPELINE (SPATIAL REASONING)
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 
[llm_decision_node-3] 🗺️  STEP 1: SPATIAL AWARENESS (2D GRID)
[llm_decision_node-3]    Local Map 360° (R=Robot, #=Obstacle, .=Clear):
[llm_decision_node-3]       ....#......###..
[llm_decision_node-3]       ...........###..
[llm_decision_node-3]       .........####...
[llm_decision_node-3]       ..........####.#
[llm_decision_node-3]       ........#...##.#
[llm_decision_node-3]       ........##..###.
[llm_decision_node-3]       ...........###..
[llm_decision_node-3]       ............##..
[llm_decision_node-3]       ........R...###.
[llm_decision_node-3]       ...............#
[llm_decision_node-3]       ...#..#..#..####
[llm_decision_node-3]       #..#......####..
[llm_decision_node-3]       .#..#...####.###
[llm_decision_node-3]       .#.......#......
[llm_decision_node-3]       .##.#...#.#.....
[llm_decision_node-3]       ................
[llm_decision_node-3]    
[llm_decision_node-3] 
[llm_decision_node-3] 📊 STEP 2: SENSOR FUSION
[llm_decision_node-3]    • LIDAR: blocked (0.2m)
[llm_decision_node-3]    • Depth: no_data
[llm_decision_node-3]    • Vision: No objects
[llm_decision_node-3]    • Scene: Unknown (DINOv2 Disabled)
[llm_decision_node-3] 
[llm_decision_node-3] 🧠 STEP 3: QWEN3 SPATIAL REASONING
[llm_decision_node-3]    • Task: "Analyze map and sensors. Output JSON."
[llm_decision_node-3]    • Context Hints: ['Status: Moving (slow)']
[llm_decision_node-3]    • Raw Response: "{"action": "GO"}"
[llm_decision_node-3]    • Inference Time: 1306ms
[llm_decision_node-3] 
[llm_decision_node-3] 🎯 STEP 4: FINAL ROBOT ACTION
[llm_decision_node-3]    • Action: STOP
[llm_decision_node-3]    • Linear Velocity: 0.0 m/s
[llm_decision_node-3]    • Angular Velocity: 0.0 rad/s
[llm_decision_node-3]    • Control Status: ENABLED
[llm_decision_node-3] 
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 
[llm_decision_node-3] [INFO] [1766993697.550847672] [llm_decision_node]: 🔧 Averaging 5 scans (metadata matches)
[llm_decision_node-3] [INFO] [1766993697.552917477] [llm_decision_node]: 🔧 Scan quality: 360 total | -132 valid (-36.7%) | 246 inf | 0 nan | 0 too_close (<0.15m) | 246 too_far (>50.0m)
[llm_decision_node-3] [INFO] [1766993697.556312080] [llm_decision_node]: 🔧 Grid: 16x16 (256 cells) | 44 obstacles (17.2% filled) | 114 valid ranges processed
[llm_decision_node-3] [INFO] [1766993698.799772338] [llm_decision_node]: 
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 🤖 AUTONOMOUS DRIVING DECISION PIPELINE (SPATIAL REASONING)
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 
[llm_decision_node-3] 🗺️  STEP 1: SPATIAL AWARENESS (2D GRID)
[llm_decision_node-3]    Local Map 360° (R=Robot, #=Obstacle, .=Clear):
[llm_decision_node-3]       ....#.....##.##.
[llm_decision_node-3]       .............##.
[llm_decision_node-3]       ..............##
[llm_decision_node-3]       ................
[llm_decision_node-3]       ...............#
[llm_decision_node-3]       .............##.
[llm_decision_node-3]       .............#..
[llm_decision_node-3]       ................
[llm_decision_node-3]       ........R.......
[llm_decision_node-3]       .##......#####..
[llm_decision_node-3]       .........#####..
[llm_decision_node-3]       #........##.#.#.
[llm_decision_node-3]       .....#..##..#.#.
[llm_decision_node-3]       .........###....
[llm_decision_node-3]       .......##.#.....
[llm_decision_node-3]       ........##..#...
[llm_decision_node-3]    
[llm_decision_node-3] 
[llm_decision_node-3] 📊 STEP 2: SENSOR FUSION
[llm_decision_node-3]    • LIDAR: caution (1.29m)
[llm_decision_node-3]    • Depth: no_data
[llm_decision_node-3]    • Vision: No objects
[llm_decision_node-3]    • Scene: Unknown (DINOv2 Disabled)
[llm_decision_node-3] 
[llm_decision_node-3] 🧠 STEP 3: QWEN3 SPATIAL REASONING
[llm_decision_node-3]    • Task: "Analyze map and sensors. Output JSON."
[llm_decision_node-3]    • Context Hints: ['Status: Moving (slow)']
[llm_decision_node-3]    • Raw Response: "{"action": "GO"}"
[llm_decision_node-3]    • Inference Time: 1240ms
[llm_decision_node-3] 
[llm_decision_node-3] 🎯 STEP 4: FINAL ROBOT ACTION
[llm_decision_node-3]    • Action: SLOW
[llm_decision_node-3]    • Linear Velocity: 0.2 m/s
[llm_decision_node-3]    • Angular Velocity: 0.0 rad/s
[llm_decision_node-3]    • Control Status: ENABLED
[llm_decision_node-3] 
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 
[llm_decision_node-3] [INFO] [1766993698.804620093] [llm_decision_node]: 🔧 Averaging 5 scans (metadata matches)
[llm_decision_node-3] [INFO] [1766993698.807819459] [llm_decision_node]: 🔧 Scan quality: 360 total | -148 valid (-41.1%) | 254 inf | 0 nan | 0 too_close (<0.15m) | 254 too_far (>50.0m)
[llm_decision_node-3] [INFO] [1766993698.810119317] [llm_decision_node]: 🔧 Grid: 16x16 (256 cells) | 49 obstacles (19.1% filled) | 106 valid ranges processed
[llm_decision_node-3] [INFO] [1766993698.810895270] [llm_decision_node]: 🗑️  Filtered 2 low-conf detections: [('cat', '0.52'), ('bench', '0.48')]
[llm_decision_node-3] [INFO] [1766993700.117821878] [llm_decision_node]: 
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 🤖 AUTONOMOUS DRIVING DECISION PIPELINE (SPATIAL REASONING)
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 
[llm_decision_node-3] 🗺️  STEP 1: SPATIAL AWARENESS (2D GRID)
[llm_decision_node-3]    Local Map 360° (R=Robot, #=Obstacle, .=Clear):
[llm_decision_node-3]       ....#....###.##.
[llm_decision_node-3]       .....#.......##.
[llm_decision_node-3]       .....##.......##
[llm_decision_node-3]       ##.............#
[llm_decision_node-3]       .#.............#
[llm_decision_node-3]       ..............#.
[llm_decision_node-3]       ...........##...
[llm_decision_node-3]       ..........#.....
[llm_decision_node-3]       ........R#......
[llm_decision_node-3]       .#.......#..#...
[llm_decision_node-3]       ##......##.#.#..
[llm_decision_node-3]       #.......##..#.#.
[llm_decision_node-3]       .........##.###.
[llm_decision_node-3]       ....#.....##....
[llm_decision_node-3]       ..........##....
[llm_decision_node-3]       .......#.#......
[llm_decision_node-3]    
[llm_decision_node-3] 
[llm_decision_node-3] 📊 STEP 2: SENSOR FUSION
[llm_decision_node-3]    • LIDAR: blocked (0.35m)
[llm_decision_node-3]    • Depth: no_data
[llm_decision_node-3]    • Vision: 2 objects detected
[llm_decision_node-3]    • Scene: Unknown (DINOv2 Disabled)
[llm_decision_node-3] 
[llm_decision_node-3] 🧠 STEP 3: QWEN3 SPATIAL REASONING
[llm_decision_node-3]    • Task: "Analyze map and sensors. Output JSON."
[llm_decision_node-3]    • Context Hints: ['Status: Moving (slow)']
[llm_decision_node-3]    • Raw Response: "{"action": "GO"}"
[llm_decision_node-3]    • Inference Time: 1303ms
[llm_decision_node-3] 
[llm_decision_node-3] 🎯 STEP 4: FINAL ROBOT ACTION
[llm_decision_node-3]    • Action: STOP
[llm_decision_node-3]    • Linear Velocity: 0.0 m/s
[llm_decision_node-3]    • Angular Velocity: 0.0 rad/s
[llm_decision_node-3]    • Control Status: ENABLED
[llm_decision_node-3] 
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 
[llm_decision_node-3] [INFO] [1766993700.123920092] [llm_decision_node]: 🔧 Averaging 5 scans (metadata matches)
[llm_decision_node-3] [INFO] [1766993700.125239641] [llm_decision_node]: 🔧 Scan quality: 360 total | -170 valid (-47.2%) | 265 inf | 0 nan | 0 too_close (<0.15m) | 265 too_far (>50.0m)
[llm_decision_node-3] [INFO] [1766993700.129626713] [llm_decision_node]: 🔧 Grid: 16x16 (256 cells) | 42 obstacles (16.4% filled) | 95 valid ranges processed
[llm_decision_node-3] [INFO] [1766993700.131224860] [llm_decision_node]: 🗑️  Filtered 2 low-conf detections: [('cat', '0.58'), ('bench', '0.52')]
[llm_decision_node-3] [INFO] [1766993701.420776301] [llm_decision_node]: 
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 🤖 AUTONOMOUS DRIVING DECISION PIPELINE (SPATIAL REASONING)
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 
[llm_decision_node-3] 🗺️  STEP 1: SPATIAL AWARENESS (2D GRID)
[llm_decision_node-3]    Local Map 360° (R=Robot, #=Obstacle, .=Clear):
[llm_decision_node-3]       .....#......#...
[llm_decision_node-3]       ...........#....
[llm_decision_node-3]       .....#..##......
[llm_decision_node-3]       ##.......##.....
[llm_decision_node-3]       .#............#.
[llm_decision_node-3]       ............##..
[llm_decision_node-3]       ............##..
[llm_decision_node-3]       .........##.....
[llm_decision_node-3]       ........R#......
[llm_decision_node-3]       .........#.#.#..
[llm_decision_node-3]       .#......######..
[llm_decision_node-3]       #.......##.#....
[llm_decision_node-3]       ........###.....
[llm_decision_node-3]       ....#....##.....
[llm_decision_node-3]       .........##.....
[llm_decision_node-3]       ..........#.....
[llm_decision_node-3]    
[llm_decision_node-3] 
[llm_decision_node-3] 📊 STEP 2: SENSOR FUSION
[llm_decision_node-3]    • LIDAR: blocked (0.2m)
[llm_decision_node-3]    • Depth: no_data
[llm_decision_node-3]    • Vision: 2 objects detected
[llm_decision_node-3]    • Scene: Unknown (DINOv2 Disabled)
[llm_decision_node-3] 
[llm_decision_node-3] 🧠 STEP 3: QWEN3 SPATIAL REASONING
[llm_decision_node-3]    • Task: "Analyze map and sensors. Output JSON."
[llm_decision_node-3]    • Context Hints: ['Status: Moving (slow)']
[llm_decision_node-3]    • Raw Response: "{"action": "GO"}"
[llm_decision_node-3]    • Inference Time: 1285ms
[llm_decision_node-3] 
[llm_decision_node-3] 🎯 STEP 4: FINAL ROBOT ACTION
[llm_decision_node-3]    • Action: STOP
[llm_decision_node-3]    • Linear Velocity: 0.0 m/s
[llm_decision_node-3]    • Angular Velocity: 0.0 rad/s
[llm_decision_node-3]    • Control Status: ENABLED
[llm_decision_node-3] 
[llm_decision_node-3] ================================================================================
[llm_decision_node-3] 
[llm_decision_node-3] [INFO] [1766993701.435054822] [llm_decision_node]: 🔧 Averaging 5 scans (metadata matches)
[llm_decision_node-3] [INFO] [1766993701.436152478] [llm_decision_node]: 🔧 Scan quality: 360 total | -268 valid (-74.4%) | 314 inf | 0 nan | 0 too_close (<0.15m) | 314 too_far (>50.0m)
[llm_decision_node-3] [INFO] [1766993701.438112457] [llm_decision_node]: 🔧 Grid: 16x16 (256 cells) | 31 obstacles (12.1% filled) | 46 valid ranges processed
[llm_decision_node-3] [INFO] [1766993701.438855673] [llm_decision_node]: 🗑️  Filtered 2 low-conf detections: [('bench', '0.58'), ('cat', '0.46')]
^C

🛑 Stopping all systems...
[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
✅ All systems stopped
^CException ignored in: <module 'threading' from '/usr/lib/python3.10/threading.py'>
Traceback (most recent call last):
  File "/usr/lib/python3.10/threading.py", line 1518, in _shutdown
    def _shutdown():
KeyboardInterrupt: 
jetson@yahboom:~/yahboomcar_ros2_ws/yahboomcar_ws/scripts$ 





● FOUND IT! The debug logs reveal the exact problem:

  Root Cause Analysis

  Critical Finding:

  🔧 Scan quality: 360 total | 269 inf | 0 nan | 0 too_close | 269 too_far

  Out of 360 LIDAR points, 269 (75%) are inf (infinite)!

  Only 91 out of 360 rays (25%) are detecting actual obstacles. This means:

  1. LIDAR is mostly seeing nothing - 75% of laser rays return no echo
  2. The 25% that DO detect change each scan - causing grid instability
  3. Temporal averaging can't help - if a ray returns inf in all 5 scans, the average is still inf

  Why Grid Changes:

  - Scan 1: 91 valid ranges → 52 obstacles (20.3% filled)
  - Scan 2: 128 valid ranges → 63 obstacles (24.6% filled)
  - Scan 3: 81 valid ranges → 27 obstacles (10.5% filled)
  - Scan 4: 120 valid ranges → 59 obstacles (23.0% filled)

  Grid fill varies 10.5% to 24.6% - a 2.3x difference!

  ---
  Why Is LIDAR Returning So Much inf?

  Possible causes:

  1. Room is too large - Max range is 50m, if walls are farther, returns inf
  2. LIDAR pointing up/down - Hitting ceiling/floor beyond range
  3. LIDAR driver broken - Initializing with inf and not updating all rays
  4. Books underneath blocking rays - Some rays hit books, some don't

  ---
  Let's Debug the LIDAR Itself

  Run this to see raw LIDAR data:

  timeout 5 ros2 topic echo /scan --once

  Look for:
  - range_max: (should be ~50.0)
  - ranges: array - count how many are inf

  If most ranges are inf, the problem is LIDAR hardware/driver, not the grid code!

  ---
  Quick Test

  Since you have the robot on books, try this:
  1. Put robot on floor (not on books)
  2. Place 4 obstacles around it (boxes, books) at 1-2m distance in N/S/E/W directions
  3. Run ./scripts/start_auto.py again

  You should see:
  - Much lower inf count (maybe 30-40% instead of 75%)
  - More stable grid
  - Consistent obstacle detection

  Want me to help debug the LIDAR driver next, or should we test with robot on floor first?






  1 +----------------------------------------------------------------------------------+
    2 |                                AUTONOMY MANAGER                                  |
    3 |                 (Central ROS2 Node Orchestrating the Control Loop)               |
    4 +----------------------------------------------------------------------------------+
    5     ^    |               ^                  |                ^                  |
    6     |    | (LLM Response)|(Prompt: "Action?")|                | (Map & Position) |
    7     |    |  "Turn left"  |                  |                |                  |
    8     |    +-------+-------+                  +----------------+--+---------------+
    9     |            |                                            |
   10 (Detections)     |                                            |
   11     |            V                                            V
   12 +--------------------------+      +------------------+      +-----------------------+
   13 |   (2) Object Detection   |      | (3) TinyLlama    |      |    (1) RTAB-Map SLAM  |
   14 |      (e.g., YOLO)        |<-----+   LLM            |<-----+ (Spatial Intelligence)|
   15 +--------------------------+      +------------------+      +-----------------------+
   16     ^                                                         ^
   17     | (Live Camera Feed)                                      | (Sensor Data)
   18     |                                                         |
   19     +---------------------------------------------------------+
   20     |
   21     V
   22 +----------------------------------------------------------------------------------+
   23 |                                     ACTION                                       |
   24 |                  (Manager Translates LLM command to Motor Control)               |
   25 +----------------------------------------------------------------------------------+
   26     |
   27     | (4. Twist Command)
   28     V
   29 +--------------------+      +-------------------+      +-----------------+
   30 |   /cmd_vel Topic   | ---> |   Motor Drivers   | ---> |  Robot Moves    |
   31 +--------------------+      +-------------------+      +-----------------+
   32                                                              |
   33                                                              | (PHYSICAL FEEDBACK)
   34                                                              V
   35                                                      [Sensors Detect Movement]
   36                                                      (Loop starts over)







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






  cd ~/yahboomcar_ros2_ws/yahboomcar_ws
  source install/setup.bash

  # Launch the autonomous driving system
  ros2 launch autonomous_driving autonomous_driving_launch.py

  # Monitor performance
  ros2 topic hz /autonomous/detections
  ros2 topic echo /autonomous/detections --once | jq '.scene.type'
  ros2 topic echo /autonomous/detections --once | jq '.detections[0].attention_score'




  # Full features (default) - RTAB-Map + AI Fusion both ON
  ./show_3d_world.py

  # Only RTAB-Map 3D mapping
  ./show_3d_world.py --no-fusion

  # Only AI Fusion Vision
  ./show_3d_world.py --no-rtabmap

  # Minimal mode (neither enabled)
  ./show_3d_world.py --no-rtabmap --no-fusion

