# CubeSatellite Adventures

This repository contains the subsystem firmware for a custom CubeSat project, focusing heavily on integrating and stabilizing the Attitude Determination and Control System (ADCS) alongside the On-Board Computer (OBC).

## Architecture & Subsystems

The system is distributed across multiple microcontrollers (Arduino Nano Every / standard Nano) to ensure stable timing and decoupled logic.

### 1. Attitude Determination and Control System (ADCS)
The ADCS is responsible for tracking the satellite's orientation and executing maneuvers to maintain a sun-facing or earth-facing attitude. 

**Thought Process & Evolution:**
*   **Binary to Proportional Control:** The original ADCS relied on binary logic (e.g., if the left sun sensor is brightest, apply maximum power counter-clockwise). This was upgraded to a **PID Controller**. By calculating the mathematical error between the left and right sun sensors, the reaction wheel now applies proportional power—slowing down as it approaches perfect alignment to avoid overshooting.
*   **Object-Oriented Refactoring:** As complexity grew, the massive loop was refactored into modular, encapsulated `structs` (`Motor`, `SunSensors`, `PIDController`, `IMUSensor`, `TemperatureSensor`). This object-oriented approach isolates hardware logic, making it easy to instantiate multiple motors or sensors in the future.
*   **IMU Integration:** An Adafruit BNO055 was integrated to track absolute 3D orientation (Euler vectors: Yaw, Pitch, Roll) and print telemetry independently from the PID tracking.
*   **Shake Override:** A safety/testing override was introduced using the IMU. By specifically reading `VECTOR_LINEARACCEL` (which automatically filters out Earth's baseline 9.8m/s² gravity), the system detects sudden external physical impacts and fires the reaction wheel to restabilize, completely interrupting the sun-tracking routine.

### 2. On-Board Computer (OBC) & Telemetry
The OBC handles high-level decision making, system-wide data pooling, and LoRa transmission to ground control.

**Thought Process & Evolution:**
*   **Interrupting Race Conditions:** A critical architectural challenge was inter-board communication. The OBC polls the ADCS for temperature (`TEMPADCS` command) over `SoftwareSerial`. Originally, the OBC checked for a response microseconds after sending the request. A deterministic timing delay was introduced, acknowledging the processing constraints of the ADCS and network latency, guaranteeing stable cross-board data flow.
*   **LoRa Packet Assembly:** The OBC aggregates local state (uptime counters, battery voltage from A0, OBC temperature) alongside the remote ADCS telemetry, assembling it into a unified string before broadcasting.

## Hardware Stack
*   **Compute:** Arduino Nano Every (ATmega4809), Arduino Nano
*   **Sensors:** BNO055 (IMU), LDRs (Sun Sensors), Analog Temperature & Voltage probes
*   **Actuators:** Single-axis Reaction Wheel driven via PWM
*   **Comm:** LoRa (430 MHz), SoftwareSerial (Board-to-Board)

## Code Structure Highlights
*   `ADCSShakeIMUReactionWheelSunSensors`: The flagship ADCS code containing the structurally encapsulated PID loop, IMU tracking, and serial parsing.
*   `OBCSendReadings`: The main OBC polling and transmission loop.

---
*Developed as part of an iterative CubeSat hardware-software integration initiative.*
