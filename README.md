# Balance Robot

This repository contains the full report and codebase for an autonomous, two-wheeled self-balancing robot. The robot is capable of maintaining its balance on two wheels, navigating an indoor environment, avoiding obstacles, and being controlled remotely via a web interface.

This project was developed by Group 19 (Jarbis) for the ELEC50015 Electronics Design Project 2 at Imperial College London.

## Features

- Self-Balancing: A cascaded PID control system implemented on an ESP32 maintains the robot's upright stability, even recovering from external pushes up to 5°.
- Autonomous Obstacle Avoidance: The robot uses a fusion of computer vision and ultrasonic ranging to navigate its environment.
	- Computer Vision: A Raspberry Pi and Pi Camera module use Monocular Visual Odometry (ORB algorithm) to detect obstacles in a wide field of view.
	- Ultrasonic Sensing: An ultrasonic sensor complements the camera by detecting objects in its close-proximity blind spot (up to 45cm).
- Web-Based User Interface: A responsive web server built with Flask and React allows for:
	- Manual Control: Real-time manual control of the robot's movement and rotation.
	- Live Video Feed: A live stream from the robot's camera.
	- Real-time PID Tuning: A developer interface to tune PID control parameters on the fly.
- Real-time Power Monitoring: An analog circuit provides data to the ESP32 to monitor the battery's voltage, current, power consumption, and estimated state of charge (SOC), all displayed on the web UI.
## System Architecture

The robot's functionality is divided across several interconnected subsystems, primarily managed by an ESP32 and a Raspberry Pi 3.

- ESP32 Microcontroller:
	- Runs the core balancing and movement control loops (PID).
	- Interfaces with the MPU6050 (Gyroscope/Accelerometer) for tilt and rotation data.
	- Controls the motors.
	- Reads data from the ultrasonic sensor.
	- Processes data from the power monitoring circuit.
	- Communicates with the Raspberry Pi (via UART) and the Web Server (via Wi-Fi).
- Raspberry Pi 3 A+:
	- Handles all computer vision tasks using a Pi Camera Module.
	- Processes camera frames to detect features (ORB), track movement (Optical Flow), and determine obstacle locations.
	- Runs the path determination logic for autonomous navigation.
	- Sends high-level commands (e.g., 'turn left', 'move forward') to the ESP32.
- Web Server:
	- Backend (Flask): Hosts the application, serves the frontend, and manages communication with the ESP32 and Raspberry Pi over a TCP socket connection.
	- Frontend (React): Provides a user-friendly interface for control and data visualization.
	- Database (SQLAlchemy): Stores the battery's state of charge between sessions.
- Power System:
	- A custom power PCB distributes power from a 7.2V NiMH battery.
	- A dedicated analog sensing circuit conditions voltage and current signals to be safely read by the ESP32's ADC.

## Technology Stack

### Hardware

- Chassis: 2-Wheeled Robot Chassis with DC Motors
- Main Controllers: ESP32, Raspberry Pi 3 Model
- Sensors:
	- MPU6050 Accelerometer/Gyroscope
	- Pi Camera Module 2
	- HC-SR04 Ultrasonic Sensor
- Power: 7.2V 2000mAh NiMH Battery, Custom Power PCB
- Circuitry: TLV2462CP Op-Amps, MCP3208 ADC
### Software & Libraries
- ESP32: C++ (PlatformIDE)
- Raspberry Pi: Python
	- Computer Vision: OpenCV
	- Communication: pyserial
- Web Server:
	- Backend: Flask (Python)
	- Frontend: React (JavaScript)
	- Database: SQLAlchemy
- Control: PID Control Theory, Complementary & Madgwick Filters
- Computer Vision Algorithms: ORB (Oriented FAST and Rotated BRIEF), Lowe's Ratio Test, Optical Flow, Triangulation

## Codebase Structure

The code for this project is organized into several branches, each corresponding to a major component of the system.

- main: The default branch containing the integrated, stable version of the project.
- Computer-vision: Contains all Python code for the Raspberry Pi's computer vision and autonomous navigation logic.
- Pid-controller: Contains the C++ firmware for the ESP32, focusing on the balancing and movement control system.
- Power: Contains the ESP32 code additions for the power monitoring subsystem.
- Website: Contains the Flask backend and React frontend for the web server.

## Getting Started

To replicate this project, you will need the hardware listed above and the corresponding code from this repository.

1. Hardware Assembly: Assemble the robot chassis, mount the ESP32, Raspberry Pi, sensors, and power monitoring circuit.
2. ESP32 Setup: Flash the ESP32 with the firmware from the Pid-controller and Power branches using the Arduino IDE or PlatformIO. Change IP to match IP of server 
3. Raspberry Pi Setup: Set up the Raspberry Pi with the code from the Computer-vision branch. Ensure all Python dependencies, like OpenCV and pyserial, are installed.
4. Server Setup: Run the Flask web server from the Website branch editing the IP of the computer hosting.
5. Power On: Connect the battery and power on the system. The ESP32 should connect to the network and the web server.
## Usage
1. Navigate to the web server's URL in your browser.
2. Use the on-screen arrow keys (or keyboard arrow keys) for manual control.
3. Observe the live camera feed and power status telemetry.
4. Switch to autonomous mode to let the robot navigate and avoid obstacles on its own.
## Team (Group 19 - Jarbis)

- Thanakorn Techachokwiwat - Power Monitoring
- Edgar Teh - Computer Vision & Autonomous Control
- Minesh Sachin Sathiyaamoorthy - Balance Control System
- Kevin Aubeeluck - Web Server & UI
- Alix Le Coguic - UART Communication
