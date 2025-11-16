# Embedded Systems Project 

This repository contains the final project developed as part of the Embedded Systems course at the University of Genoa.  
The project focuses on real-time control, sensor integration, and autonomous navigation for a buggy robot.

## Project Overview
Autonomous Buggy Firmware (Final Project)

### 1. Autonomous Buggy Firmware
**Objective**  
The objective of this project is to design an embedded control system for a buggy that can receive movement commands via UART, process them, execute corresponding motor actions, and provide feedback while ensuring safety through obstacle detection.

**Features**  
- **Command Handling**  
  - Receive movement commands in the format `$PCCMD,x,t*`, where:  
    - `x` represents the action:  
      - 1 = Forward  
      - 2 = Counterclockwise Rotation  
      - 3 = Clockwise Rotation  
      - 4 = Backward  
    - `t` specifies the duration in milliseconds.  
  - Maintain a FIFO queue (max 10 commands) for execution.  
  - Send acknowledgment messages `$MACK,1*` (success) or `$MACK,0*` (queue full).  

- **Motor Control**  
  - Four PWM signals control the left and right wheels at 10 kHz.  
  - Commands map to PWM signals to adjust speed and direction.  

- **State Management**  
  - **Waiting State**: Motors idle, LED blinks at 1 Hz, commands stored in FIFO.  
  - **Execution State**: Processes commands, executes movements, monitors sensors.  
  - **Emergency State**: Stops movement if an obstacle is detected within threshold distance, sends `$MEMRG` messages, lights blink at 1 Hz.  

- **Sensor Integration**  
  - **Battery Monitoring**: Read battery voltage from AN11 and transmit `$MBATT,v_batt*` at 1 Hz.  
  - **Distance Sensing**: Read IR sensor data from AN14/AN15, transmit `$MDIST,distance*` at 10 Hz.  
  - **Accelerometer Logging**: Acquire filtered x, y, z values at 10 Hz.  

**How to Run**  
1. Power on the buggy system.  
2. The system starts in *Waiting State*. Press RE8 to switch to *Execution State*.  
3. Send movement commands via UART.  
4. Monitor responses and sensor data through UART output.  
5. Press RE8 again to reset back to *Waiting State*.  

**Files**  
- `tools.h / tools.c`: Implements UART communication, command parsing, motor control, and sensor handling.  
- `main.c`: Manages system states, executes commands, and handles real-time constraints.  
