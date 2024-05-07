In this project, we controlled both a simulated and a real Turtlebot 3 Burger mobile robot within a realistic and challenging environment. The navigation involved several steps:


• Utilizing images from a simulated/real camera to detect and follow specific lines. For this task, we implemented a PID (Proportional Integrative Derivative) controller to maintain the robot within the roadway.
![Alt Text](Self-Driving-robot-/Results/LineFollowingRobot.gif)
• Employing a laser scan from a simulated/real LDS and the IMU (Inertial Measurement Unit) to detect and avoid obstacles while maintaining a consistent direction.
![Alt Text](Self-Driving-robot-/Results/ObstacleAvoidanceRobot.gif)
• Finally, utilizing the laser scan data along with a PID controller to navigate through a tunnel without colliding with its walls.
![Alt Text](Self-Driving-robot-/Results/TunnelCrossingRobot.gif)
