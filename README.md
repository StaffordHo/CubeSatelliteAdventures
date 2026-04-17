# CubeSatellite Adventures (V2)

This repository contains the subsystem firmware for a custom CubeSat project, successfully tested with a complete End-to-End hardware integration architecture. It demonstrates secure data transmission and active subsystem management across multiple nodes.

## End-to-End Architecture & Subsystems

The system successfully integrates three disjointed components to act synchronously:

### 1. Ground Station (GS)
The GS is responsible for interfacing with the user over a PC terminal, constructing secure tokenized queries, and catching incoming raw and encoded radio packets via LoRa.
*   **The Camera Glitch Fix:** The VC0706 Camera downloads its image buffer over the OBC's software serial connection, resulting in thousands of tiny raw binary chunks cascading via LoRa. The Ground Station was originally trying to parse these hex bytes as ASCII strings hunting for the password, throwing catastrophic syntax errors. This was resolved mathematically by pushing failed-token packages straight to the visual output buffer instead of rejecting them. 
*   **Token Routing Validation:** Successfully implements the `STAFFORDPWROX` validation check on all accepted telemetry.

### 2. On-Board Computer (OBC)
The OBC handles mid-level system aggregation and radio transit logic. 
*   **Asynchronous Pass-Through Routing:** The OBC solves previous timing bottleneck challenges using elegant delegation. If a user queries `"TEMPOBC"` from the ground, the OBC responds. If the user queries `"SUN"`, the OBC recognizes it isn't the handler for that, and blindly forwards it out of its local port to the ADCS.
*   **Decoupled Listening:** Once it routes a command to the ADCS, it frees up the loop completely, running its own tasks, only listening to the ADCS occasionally to catch its responses and immediately route them back out the LoRa antenna.

### 3. Attitude Determination and Control System (ADCS)
The ADCS acts as a fully independent "Server" responsible entirely for physical orientation metrics and motor feedback loops.
*   **Non-Blocking State Machine:** It incorporates `GPS` via NMEA mapping timeouts, `Sun Sensors`, `Temperature`, and a `Reaction Wheel` on an independent, non-blocking 50hz loop.
*   **Subsystem Security:** ADCS boots completely disabled (`ADCSStatus = 0`), requiring the explicit command `ENADCS` from ground control to engage hardware operations.
*   **Toggled Real-Time Processing:** The `GYRO` command engages real-time PID algorithm tracking across the sun sensors to orient the satellite physically in the void.

## Hardware Stack
*   **Compute:** Arduino Nano Every (ATmega4809), Standard Nanos
*   **Sensors:** BNO055 (IMU), VC0706 (Serial Camera), LDRs (Sun Sensors), GPS
*   **Actuators:** Single-axis Reaction Wheel driven via 0-255 PWM
*   **Comms:** LoRa (434 MHz P2P), Multi-band SoftwareSerial Boards

---
*Developed as part of an iterative CubeSat hardware-software integration initiative.*
