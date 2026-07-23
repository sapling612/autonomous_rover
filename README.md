# autonomous_rover
autonomous rover that can either be texted to move on its own or overridden to drive using a controller

## Features

- Autonomous obstacle avoidance
- Bluetooth controller support
- SMS remote control using a SIM7000 LTE module
- Four ultrasonic sensors for obstacle detection
- Adjustable movement and safety parameters
- EEPROM storage for calibration settings

## Hardware

- Arduino Mega 2560
- SIM7000 LTE Module
- HC-05 Bluetooth Module
- Four HC-SR04 Ultrasonic Sensors
- L298N Motor Drivers
- 4WD Rover Chassis
- 2S LiPo Battery

## Software

- Arduino IDE
- Embedded C++

## Repository Contents

- `rover_main.ino` – Main firmware for autonomous navigation and remote control
- `controller.ino` – Bluetooth controller firmware

## Future Improvements

- GPS navigation
- Camera integration
- Path planning
- Mobile app interface
