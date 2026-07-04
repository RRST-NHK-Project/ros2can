# esp32_serial_bridge

## 1. Overview

esp32_serial_bridge is an ESP32 firmware for the serial_bridge ROS 2 package.
It receives binary command frames from PC, applies them to on-board outputs, and sends feedback values back to ROS 2.

This firmware is intended for deterministic robot hardware control in NHK Project RRST systems.

---

## 2. Target Hardware

| Item | Description |
|:---|:---|
| MCU | ESP32 Dev Module |
| Framework | Arduino (PlatformIO) |
| Transport | USB Serial |
| Typical use | Actuator and sensor I/O bridge |

---

## 3. Mode Comparison (Actuators and Sensors)

Select exactly one mode in src/config.hpp.

| Mode | Purpose | DC motor | Servo | TR output | RoboMaster motor | ENC | SW | Other sensors |
|:---|:---|---:|---:|---:|---:|---:|---:|:---|
| MODE_OUTPUT | Output-only bridge | 4 | 4 | 5 | 0 | 0 | 0 | - |
| MODE_INPUT | Input-only bridge | 0 | 0 | 0 | 0 | 4 | 8 | - |
| MODE_IO | Mixed I/O bridge | 2 | 0 | 5 | 0 | 2 | 4 | - |
| MODE_ROBOMAS | RoboMaster CAN motor control | 0 | 0 | 0 | 4 | 0 | 0 | - |
| MODE_ROBOMAS_PLUS_OUTPUT | RoboMaster + full output bridge | 4 | 4 | 5 | 4 | 0 | 0 | - |
| MODE_ROBOMAS_PLUS_INPUT | RoboMaster + input bridge | 0 | 0 | 0 | 4 | 4 | 8 | - |
| MODE_ROBOMAS_PLUS_IO | RoboMaster + partial I/O bridge | 0 | 0 | 5 | 4 | 4 | 0 | - |
| MODE_DEBUG | Development/debug mode | - | - | - | - | - | - | project-specific |

Notes:
- TR output means transistor/solenoid digital output channels.
- ENC and SW values are feedback channels sent in TX frame.

---

## 4. Data Layout (int16)

RX (PC -> MCU):
- 1 to 8: motor command slots
- 9 to 16: servo command slots
- 17 to 23: TR output slots

TX (MCU -> PC):
- 1 to 8: encoder slots
- 9 to 16: switch slots
- 17 to 23: reserved

---

## 5. Configuration Items

Main items in src/config.hpp:

| Item | Description |
|:---|:---|
| DEVICE_ID | Unique ID for routing commands and feedback per MCU |
| MODE_* | Select exactly one operation mode |
| MD_PWM_FREQ | PWM frequency for DC motor driver channels |
| MD_PWM_RESOLUTION | PWM resolution (bit) for DC motor channels |
| SERVO_PWM_FREQ | PWM frequency for servo channels |
| SERVO_PWM_RESOLUTION | PWM resolution (bit) for servo channels |
| SERVOx_MIN_US / SERVOx_MAX_US | Pulse width range for each servo |
| SERVOx_MIN_DEG / SERVOx_MAX_DEG | Angle range for each servo |
| SERVOx_INIT_DEG | Initial angle for each servo |
| ENABLE_LED | Enable status LED behavior |
| ENABLE_EXTRA_TR_PIN | Optional TR6/TR7 pin usage setting |

---

## 6. Configuration Workflow

1. Open src/config.hpp.
2. Set DEVICE_ID to a unique value.
3. Enable exactly one mode macro.
4. Tune PWM and servo constants if needed.
5. Build and flash from PlatformIO.

---

## 7. Compatibility with serial_bridge (ROS 2)

This firmware is designed to pair with the serial_bridge ROS 2 node.
Keep the following aligned on both sides:

- DEVICE_ID
- Frame length and payload interpretation
- Command/feedback topic routing
- Communication period

---

## 8. Credits

Developed by NHK Project, RRST, Ritsumeikan University, Japan.
- Official Website: https://www.rrst.jp
- X (Twitter): https://x.com/RRST_BKC

![Logo](https://www.rrst.jp/img/logo.png)
