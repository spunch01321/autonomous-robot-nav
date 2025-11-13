#!/usr/bin/env python3
"""
Autonomous Robot Controller for Raspberry Pi
Implements the same navigation algorithms from the web demo
"""

import time
import numpy as np
import cv2
from rplidar import RPLidar
import RPi.GPIO as GPIO

class AutonomousRobot:
    def __init__(self):
        """Initialize robot hardware and navigation system"""
        # Hardware setup
        self.setup_motors()
        self.setup_sensors()
        
        # Navigation state
        self.position = np.array([0.0, 0.0])
        self.angle = 0.0
        self.velocity = 0.0
        
        # PID parameters
        self.kp = 2.0
        self.ki = 0.1
        self.kd = 0.5
        self.integral_error = 0.0
        self.last_error = 0.0
        
    def setup_motors(self):
        """Configure motor GPIO pins"""
        GPIO.setmode(GPIO.BCM)
        
        # Motor pins (adjust for your hardware)
        self.left_motor = {'forward': 17, 'backward': 18, 'pwm': 27}
        self.right_motor = {'forward': 22, 'backward': 23, 'pwm': 24}
        
        for motor in [self.left_motor, self.right_motor]:
            GPIO.setup(motor['forward'], GPIO.OUT)
            GPIO.setup(motor['backward'], GPIO.OUT)
            GPIO.setup(motor['pwm'], GPIO.OUT)
        
        # PWM for speed control (1000 Hz)
        self.left_pwm = GPIO.PWM(self.left_motor['pwm'], 1000)
        self.right_pwm = GPIO.PWM(self.right_motor['pwm'], 1000)
        
        self.left_pwm.start(0)
        self.right_pwm.start(0)
    
    def setup_sensors(self):
        """Initialize LIDAR and camera"""
        # LIDAR
        self.lidar = RPLidar('/dev/ttyUSB0')
        
        # Camera
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    def get_lidar_scan(self):
        """Get one complete LIDAR scan"""
        for scan in self.lidar.iter_scans():
            return np.array([(angle, distance) for quality, angle, distance in scan])
    
    def get_camera_frame(self):
        """Capture camera frame"""
        ret, frame = self.camera.read()
        return frame if ret else None
    
    def detect_obstacles(self, lidar_scan, camera_frame):
        """Detect obstacles from sensor fusion"""
        obstacles = []
        
        # Process LIDAR data
        for angle, distance in lidar_scan:
            if distance < 1000:  # Within 1 meter
                x = distance * np.cos(np.radians(angle)) / 1000
                y = distance * np.sin(np.radians(angle)) / 1000
                obstacles.append({'x': x, 'y': y, 'radius': 0.1})
        
        # TODO: Add computer vision object detection
        # Can use YOLO, MobileNet, or other CV models
        
        return obstacles
    
    def a_star(self, start, goal, obstacles):
        """A* pathfinding algorithm"""
        # Same implementation as web demo
        # Simplified for brevity - use full version from simulation
        
        grid_size = 0.2  # 20cm resolution
        # ... (implement full A* algorithm)
        
        return []  # Return path waypoints
    
    def pid_control(self, target_angle, dt):
        """PID controller for heading"""
        error = target_angle - self.angle
        
        # Normalize angle to [-pi, pi]
        while error > np.pi:
            error -= 2 * np.pi
        while error < -np.pi:
            error += 2 * np.pi
        
        # PID terms
        self.integral_error += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        
        output = (self.kp * error + 
                 self.ki * self.integral_error + 
                 self.kd * derivative)
        
        self.last_error = error
        
        return output
    
    def set_motor_speeds(self, left_speed, right_speed):
        """Set motor speeds (-100 to 100)"""
        # Left motor
        if left_speed > 0:
            GPIO.output(self.left_motor['forward'], GPIO.HIGH)
            GPIO.output(self.left_motor['backward'], GPIO.LOW)
            self.left_pwm.ChangeDutyCycle(abs(left_speed))
        else:
            GPIO.output(self.left_motor['forward'], GPIO.LOW)
            GPIO.output(self.left_motor['backward'], GPIO.HIGH)
            self.left_pwm.ChangeDutyCycle(abs(left_speed))
        
        # Right motor (similar)
        if right_speed > 0:
            GPIO.output(self.right_motor['forward'], GPIO.HIGH)
            GPIO.output(self.right_motor['backward'], GPIO.LOW)
            self.right_pwm.ChangeDutyCycle(abs(right_speed))
        else:
            GPIO.output(self.right_motor['forward'], GPIO.LOW)
            GPIO.output(self.right_motor['backward'], GPIO.HIGH)
            self.right_pwm.ChangeDutyCycle(abs(right_speed))
    
    def navigate_to_goal(self, goal):
        """Main navigation loop"""
        path = self.a_star(self.position, goal, [])
        
        for waypoint in path:
            while np.linalg.norm(waypoint - self.position) > 0.1:
                # Get sensor data
                lidar_scan = self.get_lidar_scan()
                camera_frame = self.get_camera_frame()
                obstacles = self.detect_obstacles(lidar_scan, camera_frame)
                
                # Calculate heading to waypoint
                dx = waypoint[0] - self.position[0]
                dy = waypoint[1] - self.position[1]
                target_angle = np.arctan2(dy, dx)
                
                # PID control
                dt = 0.05  # 20 Hz control loop
                steering = self.pid_control(target_angle, dt)
                
                # Differential drive
                base_speed = 50  # 0-100 scale
                left_speed = base_speed - steering * 20
                right_speed = base_speed + steering * 20
                
                # Clip speeds
                left_speed = np.clip(left_speed, -100, 100)
                right_speed = np.clip(right_speed, -100, 100)
                
                self.set_motor_speeds(left_speed, right_speed)
                
                time.sleep(dt)
        
        # Stop at goal
        self.set_motor_speeds(0, 0)
    
    def cleanup(self):
        """Clean shutdown"""
        self.set_motor_speeds(0, 0)
        self.lidar.stop()
        self.lidar.disconnect()
        self.camera.release()
        GPIO.cleanup()

if __name__ == "__main__":
    robot = AutonomousRobot()
    
    try:
        # Navigate to goal (2m forward, 1m right)
        goal = np.array([2.0, 1.0])
        robot.navigate_to_goal(goal)
    except KeyboardInterrupt:
        print("Stopping robot...")
    finally:
        robot.cleanup()