# RoboKeeper ⚽🤖
An autonomous tabletop robotic goalkeeper using computer vision and feedback control.

---

## Problem Statement

Design and implement an autonomous robotic goalkeeper that:

- Detects a moving ball using computer vision,  
- operates in real time under constraints, and sensor noise
- Uses feedback control (PID) to actuate a motorized platform to intercept the ball before it reaches the goal.
  
---

## Hardware Setup

<p align="center">
  <img src="images/Hardware_setup.png" width="600">
</p>

<p align="center">
  <em>Figure 1: Tabletop RoboKeeper hardware setup showing the camera, actuator, and goal region.</em>
</p>

## System Architecture 
<p align="center">
  <img src="images/Sys_Arc.png" width="750">
</p>

<p align="center">
  <em>Figure 2: Software and hardware pipeline for real-time ball tracking and motor control.</em>
</p>

The system is split between a high-level perception and planning pipeline running in Python and a low-level control loop implemented on an Arduino microcontroller.

The webcam streams frames to the Python process, where HSV color filtering is used to detect the ball. From the detected ball position, a target servo angle is computed corresponding to the predicted interception point at the goal plane. This target value is transmitted to the Arduino over a serial connection.

On the microcontroller, a PID controller regulates the motor position to track the commanded target angle, enabling fast and stable interception of incoming shots.

## Requirements

### Software
- Python 3.x  
- OpenCV (`cv2`)  
- NumPy  
- Matplotlib  
- Arduino IDE  

### Hardware
- USB camera (~100 FPS recommended)  
- DC motor  
- Motor driver  
- Arduino Uno  
- Jumper wires  
- Ping pong ball (target object)  
- Tabletop field and goal frame / net  
- 3D-printed goalie  
- 3D-printed motor mount (for securing the motor)  
- Rectifier (for motor driver power conditioning)

