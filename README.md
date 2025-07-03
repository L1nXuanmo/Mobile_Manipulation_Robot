# 🤖 Autonomous Mobile Manipulation System – Group Project Fork

A fork of a collaborative robotics project focused on mobile manipulation, perception, and task execution in a semi-structured environment.  
I was responsible for and contributed to the following components:

---

## 🔍 Perception & Object Detection
- 📦 Implemented deep learning–based object detection using Intel RealSense RGB-D data  
- 🧠 Integrated ArUco marker detection for precise localization of objects and boxes  
- 🔁 Designed object de-duplication via bounding box distance thresholding  

---

## 🤖 Arm Control & Grasping
- 🦾 Developed a vision-based servoing pipeline for the robotic arm (no IK required)  
- 🎯 Implemented closed-loop control using HSV color filtering & edge detection  
- 🧸 Designed dual-strategy grasping for cube/sphere (color) and fluffy objects (edge + DL)  
- ⚙️ Built a Cartesian controller with PI control loop for smooth motor coordination  
- 🎛️ Controlled multi-axis motors (ID 3–6) with centroid-guided tracking and yaw alignment  

---

## 📍 Localization & Navigation
- 🧭 Integrated ICP-based localization with wheel odometry fusion  
- 🗺️ Contributed to multi-sensor odometry and TF transform debugging  
- 🧱 Supported A* path planning and waypoint-based trajectory following  

---

## 🧠 System Integration & Strategy
- 🧩 Contributed to the final behavior tree logic and goal/task switching  
- 🎮 Developed joystick teleoperation interface for manual override and debugging  
- 🔗 Integrated all modules: detection, localization, planning, arm control, and system orchestration  

---

## 🛠️ Technologies Used
- ROS (Robot Operating System)
- Python & C++
- OpenCV, TensorFlow/PyTorch (for DL detection)
- RealSense SDK, ArUco, LiDAR, Phidgets Motor Controller
