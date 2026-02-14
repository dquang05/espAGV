# AGV Control and Lidar Streaming System

## Overview
This project implements an advanced AGV (Automated Guided Vehicle) control system with integrated lidar streaming capabilities. The system is designed to operate on an ESP32 microcontroller and leverages FreeRTOS for multitasking. It provides real-time control, telemetry, and communication features, making it suitable for robotics and IoT applications.

## Features
- **Lidar Integration**: Real-time lidar data streaming over TCP (port 9000).
- **IMU Telemetry**: High-frequency telemetry data from an MPU6050 IMU sensor over TCP (port 9002).
- **Control Protocol**: Supports multiple control modes (manual and autonomous) via a TCP control server (port 9001).
- **UDP Beacon**: Periodic UDP beacon for network discovery (port 50000).
- **WiFi Power Management**: Dynamic power management to optimize energy consumption.
- **FreeRTOS Multitasking**: Efficient task scheduling for lidar, IMU, control, and beacon functionalities.

## System Architecture
The system is built around the ESP32 microcontroller and utilizes the following components:

- **Lidar Sensor**: Provides distance measurements for navigation.
- **MPU6050 IMU**: Measures acceleration and angular velocity for motion tracking.
- **WiFi Communication**: Enables remote control and data streaming.
- **FreeRTOS**: Manages concurrent tasks for real-time performance.

## Control Modes
1. **Manual Mode**: Direct control of the AGV's speed and direction.
2. **Autonomous Mode**: Executes pre-defined motion commands (distance, angle, velocity).

## Communication Protocols
- **TCP**:
  - Lidar data streaming: `tcp://<IP>:9000`
  - IMU telemetry: `tcp://<IP>:9002`
  - Control commands: `tcp://<IP>:9001`
- **UDP**:
  - Beacon for network discovery: `udp://<IP>:50000`

## Tasks
The system uses FreeRTOS tasks to manage its functionalities:
- **Lidar Task**: Streams lidar data and handles connection management.
- **IMU Telemetry Task**: Reads IMU data, processes it, and sends telemetry packets.
- **Control Task**: Handles incoming control commands and updates the AGV's state.
- **Autonomous Task**: Processes and executes autonomous motion commands.
- **Beacon Task**: Sends periodic UDP beacons for network discovery.

## Setup Instructions
1. **Hardware Requirements**:
   - ESP32 microcontroller
   - Lidar sensor
   - MPU6050 IMU sensor
   - Motor driver and motors
2. **Software Requirements**:
   - PlatformIO IDE
   - Arduino framework
3. **Configuration**:
   - Update WiFi credentials in the `main.cpp` file:
     ```cpp
     static const char *WIFI_SSID = "Your_SSID";
     static const char *WIFI_PASS = "Your_PASSWORD";
     ```
   - Configure UART pins and other hardware settings as needed.
4. **Build and Upload**:
   - Open the project in PlatformIO.
   - Build and upload the firmware to the ESP32.
5. **Run**:
   - Monitor the serial output for debugging information.
   - Connect to the specified TCP/UDP ports for data streaming and control.

## Future Enhancements
- Add lidar probing for fault detection.
- Implement advanced motion planning algorithms.
- Enhance security for WiFi communication.

## Acknowledgments
Special thanks to the open-source community for providing libraries and tools that made this project possible.