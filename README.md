# 🚗 Advanced Driver Assistance System (ADAS) Prototype with TI's MSP430FR5969

This project is a **Level 2 ADAS prototype** built using the **Texas Instruments' MSP430FR5969** microcontroller.  
It demonstrates three key ADAS features implemented at a prototype scale:  

- **Lane Control (Line Following)** - Using IR sensors to detect and follow a track (simulating lane assist).  
- **Adaptive Cruise Control** - Using an ultrasonic sensor to monitor distance from obstacles/vehicles ahead and automatically adjust speed or stop when needed.
- **Collision Detection & Prevention** - Using IR sensors to detect collisions before they happen and execute preventive measures

---

## 🔧 Hardware Stack
- **TI's MSP430FR5969** LaunchPad (main controller)  
- **IR Sensors (4x)** - for line detection and lane control & Collision detection 
- **Ultrasonic Sensor (HC-SR04)** – for adaptive cruise / collision avoidance  
- **DC Motors + Motor Driver (L298N 12V / equivalent)** - for car movement  
- **Power Supply** - battery-powered prototype car chassis  

---
