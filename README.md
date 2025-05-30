# Autonomy in Action with the Arduino Alvik
Repository of the code used to develop a line following robot with color recognition. Developed by Semih Eski and Brendan Leung, students at Dawson College during the Winter 2025 Semester.
# About This Project
This is an open-source autonomous vehicle project that uses an Arduino Alvik. It extends beyond Alvik's out-of-box capabilities by bringing together a Raspberry Pi 3B+, used for computer vision, and a 3D-printed 5-DOF robotic arm. Alvik leverages a PID algorithm and its infrared sensors to do line following, and its color-recognition sensor to detect colors underneath. A Raspberry Pi and a Pi camera are mounted on a 3D-printed stable makeshift holder that is secured through Alvik's screw threads. When the Alvik detects the color white underneath, it stops and commands, through serial communication, the RPi to run a computer vision program to calculate the distance from Alvik to a ping pong ball placed ahead. The RPi returns the distance to Alvik who accurately calculates time to travel at a fixed RPM, drives and stops next to the ball. The RPi then controls the robotic arm to pick up the ball through the PCA9685 servo board using inverse kinematics. Alvik continues travelling around the track until it detects the color orange, placed next to a basket, in which the robotic arm drops the ping pong ball. Alvik continues moving around the track until it is stopped.
# Features
* PID line following
* HSV color detection
* Touch sensor
* Serial communication
* Computer vision color recognition
* Computer vision distance calculation
* Inverse kinematics
# Hardware Requirements
* Arduino Alvik
* Raspberry Pi 3B+
* 3D printed robotic arm
* PCA9685 servo board
* Raspberry Pi camera module 1
* Jumper wires
* Black electrical tape
* 8xM3x15mm screws
* 6xFS90 micro servos
* 5V/2.5A+ portable power supply
* 5V/5A USB battery pack
# Files
* `serialcomm.py`: Main RPi sketch for serial communication, computer ball recognition and vision distance calculation
* `snap_photo.py`: Captures a photo through the RPi camera
* `distancecalc.py`: RPi sketch for only calculating distance
* `ball_color_detection.py`: RPi sketch for only ball detectiong and drawing a contour
* `line_follower.ino`: Main Arduino sketch for line following, HSV color recognition and serial communication
* `robot_arm_parts`: All the parts to 3D print in .SLDPRT format
# Getting Started
## Prerequisites
- 
