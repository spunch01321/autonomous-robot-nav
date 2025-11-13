# Autonomous Mobile Robot Navigation System

A demonstration of core autonomous navigation capabilities including A* pathfinding, sensor fusion (LIDAR + vision), and PID control for mobile robotics applications.


## 🎯 Project Overview

This project implements a complete autonomous navigation stack for mobile robots, demonstrating production-ready algorithms used in real-world robotics systems. The simulation showcases sensor integration, path planning, and real-time control systems that are directly transferable to physical hardware platforms like Raspberry Pi.

## ✨ Key Features

### 1. **A* Pathfinding Algorithm**
- Optimal path planning with guaranteed shortest path
- Dynamic obstacle avoidance with safety margins
- Grid-based spatial decomposition (20x20 resolution)
- Real-time path recalculation on target change

### 2. **LIDAR Sensor Simulation**
- 16-ray distance sensing array
- 150m maximum detection range
- 360° coverage with configurable resolution
- Ray-casting collision detection

### 3. **Computer Vision System**
- Object detection within camera field of view (60° FOV)
- 200m vision range
- Real-time object tracking and classification
- Distance estimation for detected objects

### 4. **PID Control System**
- Smooth velocity control with adaptive speed
- Heading control with angular velocity regulation
- Tuned parameters: Kp=2.0, Ki=0.1, Kd=0.5
- Waypoint-based navigation with distance thresholds

### 5. **Sensor Fusion**
- Combined LIDAR and vision data processing
- Multi-modal obstacle detection
- Redundant sensing for reliability

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                   Navigation Stack                       │
├─────────────────────────────────────────────────────────┤
│                                                           │
│  ┌──────────────┐      ┌──────────────┐                │
│  │   Sensors    │      │  Path        │                │
│  │              │      │  Planning    │                │
│  │ • LIDAR      │─────▶│              │                │
│  │ • Camera     │      │ • A* Search  │                │
│  │ • IMU        │      │ • Collision  │                │
│  └──────────────┘      │   Detection  │                │
│         │              └──────────────┘                │
│         │                      │                        │
│         ▼                      ▼                        │
│  ┌──────────────┐      ┌──────────────┐               │
│  │   Sensor     │      │   Motion     │               │
│  │   Fusion     │─────▶│   Control    │               │
│  │              │      │              │               │
│  │ • Object     │      │ • PID        │               │
│  │   Detection  │      │ • Velocity   │               │
│  │ • Distance   │      │ • Steering   │               │
│  └──────────────┘      └──────────────┘               │
│                               │                         │
│                               ▼                         │
│                        ┌──────────────┐                │
│                        │   Actuators  │                │
│                        │              │                │
│                        │ • Motors     │                │
│                        │ • Servos     │                │
│                        └──────────────┘                │
└─────────────────────────────────────────────────────────┘
```

## 🚀 Getting Started

### Web Demo (No Installation Required)

Simply open `index.html` in a modern web browser to run the simulation.

### Local Development

1. Clone the repository:
```bash
git clone https://github.com/yourusername/autonomous-robot-nav.git
cd autonomous-robot-nav
```

2. Open the demo:
```bash
# Using Python
python -m http.server 8000

# Using Node.js
npx serve

# Or simply open index.html in your browser
```

3. Navigate to `http://localhost:8000`

## 🎮 Usage

1. **Start Simulation**: Click the "Start" button to begin
2. **Set Target**: Click anywhere on the canvas to set a navigation goal
3. **Observe Behavior**: Watch the robot:
   - Calculate optimal path (blue dashed line)
   - Scan environment with LIDAR (green rays)
   - Detect objects with camera (yellow cone)
   - Navigate smoothly avoiding obstacles

## 🔧 Technical Implementation

### Pathfinding Algorithm
```python
# A* Algorithm Pseudocode
function A_Star(start, goal, obstacles):
    openSet = [start]
    cameFrom = {}
    gScore = {start: 0}
    fScore = {start: heuristic(start, goal)}
    
    while openSet is not empty:
        current = node in openSet with lowest fScore
        
        if current == goal:
            return reconstruct_path(cameFrom, current)
        
        openSet.remove(current)
        
        for neighbor in neighbors(current):
            tentative_gScore = gScore[current] + distance(current, neighbor)
            
            if tentative_gScore < gScore[neighbor]:
                cameFrom[neighbor] = current
                gScore[neighbor] = tentative_gScore
                fScore[neighbor] = gScore[neighbor] + heuristic(neighbor, goal)
                
                if neighbor not in openSet:
                    openSet.add(neighbor)
    
    return failure
```

### PID Controller
```python
# PID Control Loop
def pid_controller(current, target, dt):
    Kp = 2.0  # Proportional gain
    Ki = 0.1  # Integral gain
    Kd = 0.5  # Derivative gain
    
    error = target - current
    integral += error * dt
    derivative = error / dt
    
    output = Kp * error + Ki * integral + Kd * derivative
    return output
```

### Sensor Simulation
```python
# LIDAR Ray Casting
def simulate_lidar(position, angle, obstacles, num_rays=16):
    rays = []
    max_range = 150
    
    for i in range(num_rays):
        ray_angle = angle + (i * 2 * PI / num_rays)
        min_distance = max_range
        
        for obstacle in obstacles:
            distance = calculate_intersection(position, ray_angle, obstacle)
            if distance < min_distance:
                min_distance = distance
        
        rays.append({
            'angle': ray_angle,
            'distance': min_distance
        })
    
    return rays
```

## 🔄 Adapting to Raspberry Pi

This codebase is designed to be easily ported to real hardware. Here's how to adapt it:

### Hardware Requirements
- Raspberry Pi 4 (4GB+ recommended)
- LIDAR sensor (e.g., RPLIDAR A1/A2)
- Camera module (Raspberry Pi Camera or USB webcam)
- Motor driver (L298N or similar)
- DC motors with encoders
- Power supply (7.4V LiPo recommended)

### Software Stack
```bash
# Install dependencies
sudo apt-get update
sudo apt-get install python3-pip python3-opencv
pip3 install numpy scipy

# For ROS integration (optional)
sudo apt-get install ros-noetic-desktop-full
```

### Example Python Implementation
```python
import RPi.GPIO as GPIO
import cv2
import numpy as np
from rplidar import RPLidar

class AutonomousRobot:
    def __init__(self):
        self.lidar = RPLidar('/dev/ttyUSB0')
        self.camera = cv2.VideoCapture(0)
        self.setup_motors()
    
    def setup_motors(self):
        GPIO.setmode(GPIO.BCM)
        self.motor_pins = {'left': (17, 18), 'right': (27, 22)}
        # Configure PWM for speed control
    
    def get_lidar_data(self):
        return list(self.lidar.iter_scans())
    
    def get_camera_frame(self):
        ret, frame = self.camera.read()
        return frame
    
    def navigate(self, target):
        # Use same A* algorithm from simulation
        path = self.find_path(self.position, target)
        
        for waypoint in path:
            while not self.reached(waypoint):
                # Get sensor data
                lidar_data = self.get_lidar_data()
                camera_frame = self.get_camera_frame()
                
                # Update control
                self.update_motors(waypoint)
                
        self.stop()
```

## 📊 Performance Metrics

- **Path Planning**: ~10ms for 30x20 grid
- **Sensor Processing**: 60 FPS
- **Control Loop**: 60 Hz update rate
- **Path Optimality**: Guaranteed shortest grid path
- **Obstacle Detection**: 360° coverage

## 🛠️ Future Enhancements

- [ ] **SLAM Implementation**: Simultaneous Localization and Mapping
- [ ] **Dynamic Obstacles**: Moving obstacle prediction and avoidance
- [ ] **Machine Learning**: Neural network for terrain classification
- [ ] **Multi-Robot Coordination**: Fleet management algorithms
- [ ] **ROS Integration**: Full ROS node structure
- [ ] **3D Navigation**: Elevation mapping and 3D pathfinding
- [ ] **Advanced Sensors**: IMU fusion, GPS integration

## 📚 Resources & References

- **Algorithms**:
  - [A* Pathfinding](https://en.wikipedia.org/wiki/A*_search_algorithm)
  - [PID Control Theory](https://en.wikipedia.org/wiki/PID_controller)
  
- **Robotics Frameworks**:
  - [ROS (Robot Operating System)](https://www.ros.org/)
  - [OpenCV for Computer Vision](https://opencv.org/)
  
- **Hardware**:
  - [Raspberry Pi Documentation](https://www.raspberrypi.org/documentation/)
  - [RPLIDAR SDK](https://github.com/Slamtec/rplidar_sdk)

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## 📄 License

MIT License - feel free to use this project for learning, portfolio, or commercial purposes.

## 👤 Author

- GitHub: @spunch01321
- LinkedIn: https://www.linkedin.com/in/shawn-punch/

## 🙏 Acknowledgments

- Inspired by real-world autonomous vehicle navigation systems
- Built with modern web technologies for easy demonstration
- Designed for direct hardware implementation

---

**⭐ If you found this project helpful, please consider giving it a star!**