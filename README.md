# Mobile Tool Assistant

An autonomous mobile robot that follows a worker and carries tools in indoor industrial environments.

# Tools Used

## Hardware
- STM32F407DISCO Development Board
- ESP8266MOD Microcontroller (Remote node)
- 2x DWM1000 UWB Modules (User tracking)
- 3x VL53L1X ToF Sensors (Obstacle detection)
- L298N Motor Driver
- 2x TT DC Gear Motors
- Buck Converter Module
- Rechargeable AA NiMH Batteries (8x)
- BambuLab P1S 3D Printer (Chassis)
- Multimeter, Soldering Iron, Jumper Wires, Protoboard

## Software
- STM32CubeIDE v1.19.0
- Arduino IDE v2.3.7
- FreeRTOS (Real-time OS on STM32)
- Serial Debug Assistant
- GitHub (Version control)


## Repository Structure
```
MobileToolAssistant
├── DWM1000_tests/ # Iterative UWB development tests 
├── MobileToolAssistant/ # Final source code
│ ├── RemoteEnd/ # User remote node (ESP8266)
│ └── RobotEnd/ # Robot platform (STM32)
├── docs/ # Documentation and CAD files
└── README.md
```


## Quick Start

1. **Hardware setup**: See [User Manual](./docs/user_manual.pdf) for assembly, battery installation, and power-on steps.
2. **Build & flash**:
   - Robot: STM32CubeIDE project in `MobileToolAssistant/RobotEnd/`
   - Remote: Arduino IDE sketch in `MobileToolAssistant/RemoteEnd/`
3. **Operation**: Press the STANDBY button on the remote to start/stop following.

## Key Features

- UWB‑based user tracking (1m ± 0.5m following distance)
- ToF obstacle detection and avoidance
- Remote standby control
- Payload capacity: 1 kg
- Max following speed: 6 km/h

## Overall Block Diagram 
<img width="604" height="350" alt="Screenshot_1" src="https://github.com/user-attachments/assets/1afb5947-a75a-4c29-9746-3b261ed1421b" /> 


## Documentation

- [Requirements Specification](./docs/appendices/requirements_specification.pdf)
- [System Specification](./docs/appendices/system_specification.pdf)
- [Project Management](./docs/appendices/project_management.pdf)
- [User Manual](./docs/appendices/user_manual.pdf)
- [Final Report](./docs/final_report.pdf)

## Team

- Aneek Mohamed Ruksan
- Aditya Ramakrishnan
- Ethan D'Almeida
- Chidubem Emeka-Nwuba
