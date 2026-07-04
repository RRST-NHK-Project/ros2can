# esp32_s3_serial_bridge

## 1. Overview

esp32_s3_serial_bridge is an ESP32-S3 firmware for the serial_bridge ROS 2 package.
It provides binary serial communication with PC and executes hardware tasks based on the selected mode.

This variant targets RRST-ESP32-S3 based boards and focuses on compact actuator/sensor integration.

---

## 2. Target Hardware

| Item | Description |
|:---|:---|
| MCU | RRST-ESP32-S3 Rev.1 |
| Framework | Arduino (PlatformIO) |
| Transport | USB Serial |
| Typical use | Integrated I/O and RoboMaster control |

---

## 3. Mode Comparison (Actuators and Sensors)

Select exactly one mode in src/config.hpp.

| Mode | Purpose | DC motor | Servo | TR output | RoboMaster motor | ENC | SW | Other sensors |
|:---|:---|---:|---:|---:|---:|---:|---:|:---|
| MODE_IO | Integrated local I/O control | 4 | 4 | 6 | 0 | 2 | 5 | - |
| MODE_ROBOMAS | RoboMaster CAN motor control | 0 | 0 | 0 | 4 | 0 | 0 | - |
| MODE_ROBOMAS_AD | RoboMaster + additional output task | 4 | 4 | 6 | 4 | 0 | 0 | - |
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
- Encoder and switch feedback channels are mode-dependent

---

## 5. Configuration Workflow

1. Open src/config.hpp.
2. Set DEVICE_ID.
3. Enable exactly one mode macro.
4. Adjust PWM and servo parameters if required.
5. Build and flash with PlatformIO.

---

## 6. Compatibility with serial_bridge (ROS 2)

This firmware works with the serial_bridge ROS 2 node.
Ensure the following are consistent across PC and MCU:

- DEVICE_ID
- Payload layout (Rx_16Data and Tx_16Data interpretation)
- Topic routing on ROS 2 side
- Control/feedback cycle timing

---

## 7. Credits

Developed by NHK Project, RRST, Ritsumeikan University, Japan.
- Official Website: https://www.rrst.jp
- X (Twitter): https://x.com/RRST_BKC

![Logo](https://www.rrst.jp/img/logo.png)
