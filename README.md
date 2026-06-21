# ESP32-S3 ZVD Pick-and-Place Robot Arm

ESP32-S3 기반 ZVD(Zero Vibration Derivative) 입력 성형 및 역기구학 적용 진공 픽앤플레이스 로봇팔 펌웨어

## Hardware
- **MCU**: ESP32-S3 DevKitC (Dual Core, FreeRTOS, Wi-Fi AP)
- **Robot Arm**: [Community Robot Arm](https://github.com/20sffactory/community_robot_arm) (3DOF)
- **Motor Driver**: TB6600 x3 (24V SMPS)
- **Sensor**: MPU6050 (I2C accelerometer/gyroscope)
- **Vacuum**: 6V Vacuum Pump + Solenoid Valve (5V SMPS, Relay Module)
- **Limit Switches**: 3D Printer Endstops (NC/NO configurable)

## Features
- **Web GUI Dashboard** (Wi-Fi AP: `RobotArm_ZVD_WiFi` / `192.168.4.1`)
  - Real-time status monitoring (MPU6050 6-axis, limit switches, relay)
  - Cartesian XYZ jog with linear interpolation (Forward Kinematics)
  - Joint motor jog (individual axis control)
  - Pick & Place cycle control with configurable A/B coordinates
  - Pin map visualization
- **ZVD Input Shaping** — vibration-free motion via auto-tuned natural frequency
- **4-Stage Homing** — backoff → approach → precision re-approach → offset move
- **Inverse/Forward Kinematics** — real-time XYZ position tracking
- **FreeRTOS Dual Core** — Core 0: state machine, Core 1: stepper motion + web server

## Based On
- Robot Arm: [20sffactory/community_robot_arm](https://github.com/20sffactory/community_robot_arm)
- Original Design: [ftobler/robotArm](https://github.com/ftobler/robotArm)
