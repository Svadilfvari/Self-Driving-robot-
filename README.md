# 🤖 Autonomous Rescue Robot – Multi-Sensor ROS Navigation  
🛠️ *Python, ROS1, OpenCV, PID Control, Embedded Systems*

This project implements a robust ROS-based autonomous navigation system for the **TurtleBot3 Burger**, capable of operating in both simulated and real environments. The system fuses multi-modal sensor data—camera, LiDAR, and IMU—to perform **lane tracking**, **obstacle avoidance**, and **tunnel traversal**.

---

## 🔧 Key Features and Achievements

- ✅ Developed a **modular ROS architecture** for autonomous navigation using:
  - `ImageProcessingNode` (camera-based lane detection)
  - `LaserControlStrategy` (LiDAR-based obstacle analysis)
  - `LineFollowingNode` (decision-making and control logic)
- 🎯 Tuned a **PID controller** to maintain lane tracking with a **tracking error within ±15 pixels**
- 🚧 Achieved **100% obstacle avoidance success rate** across real-world trials using dynamic velocity adjustment
- 🕳️ Achieved **100% tunnel traversal success rate**, maintaining central alignment using a **PD wall-following strategy**
- 📉 Suppressed derivative noise in PID control through **histogram-guided tuning**
- ⏱️ Proposed a **phase-advance filter** to reduce system latency and enhance control responsiveness
- 📷 Integrated **real-time image processing** (OpenCV) for lane detection and tunnel entry recognition
- 🔄 Dynamic behavior switching based on context: line following, obstacle avoidance, or tunnel mode

---

## 🧠 Navigation Workflow

### 1. Lane Following with PID Control
- **Sensor**: RGB Camera  
- **Task**: Detect lane centroids; follow the path using PID correction  
- **Performance**: Tracking error maintained within ±15 px

![Line Following Robot](https://github.com/Svadilfvari/Self-Driving-robot-/blob/main/Results/LineFollowingRobot.gif)


---

### 2. Obstacle Avoidance via LiDAR and IMU
- **Sensors**: LDS LiDAR + IMU  
- **Task**: Dynamically avoid obstacles using velocity modulation based on laser scan analysis  
- **Result**: 100% obstacle clearance rate in both simulation and real-world environments

![Line Following Robot](https://github.com/Svadilfvari/Self-Driving-robot-/blob/main/Results/ObstacleAvoidanceRobot.gif)

---

### 3. Tunnel Traversal with Wall Following
- **Trigger**: Blue strip detection (real robot) or positional threshold (sim)  
- **Control**: PD controller guided by side laser scans and IMU stabilization  
- **Outcome**: 100% success rate in tunnel crossing without wall collision

![Tunnel Crossing](https://github.com/Svadilfvari/Self-Driving-robot-/blob/main/Results/TunnelCrossingRobot.gif)

---

## 🧩 ROS Node Architecture


+-----------------------+       +---------------------+
|  ImageProcessingNode  | --->  |   /lanesCentroids   |
+-----------------------+       +---------------------+
            |                               |
            v                               v
+-----------------------+       +---------------------+
|  LaserControlStrategy | --->  | /meanDistancesLaser |
+-----------------------+       +---------------------+
                                             |
                                             v
                                +-------------------------+
                                |   LineFollowingNode     |
                                |   (PID / Avoid / Tunnel)|
                                +-------------------------+
# 📊 Control Insights & System Tuning

- PID tuning was based on real-time error trends and statistical histograms  
- **Correlation with error:**
  - **Proportional:** 1.0 (strong linear relationship)  
  - **Integral:** ~0.095 (accumulates bias over time)  
  - **Derivative:** ~0.067 (sensitive to sudden changes)  
- Histogram analysis revealed performance phases and guided suppression of derivative noise  
- ✅ **Achieved 100% tunnel traversal success rate**  
- ✅ **Achieved 100% obstacle avoidance efficiency** in physical tests  
- Tuned PID controller to maintain **tracking error within ±15 pixels**, with **derivative noise suppressed** using histogram-guided tuning

---

## 📌 Summary

This system demonstrates reliable autonomous behavior in complex indoor navigation scenarios, including narrow passages and obstacle-dense tracks. The project illustrates the power of **multi-sensor fusion**, **modular ROS design**, and **data-driven control tuning** for real-world robotics.
