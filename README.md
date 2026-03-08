# ESP32 BLE Line Following Car

ESP32-based autonomous line following robot using **CNY70 infrared
sensors**, **L293D motor drivers**, and **Bluetooth Low Energy (BLE)**
communication.

------------------------------------------------------------------------

## 🚗 Project Overview

<p align="center">
  <img src="https://github.com/user-attachments/assets/a2e31e32-df83-4851-ad54-892a1847c068" width="300"/>
</p>

This project implements a **line-following robotic car** controlled by
an **ESP32 microcontroller**.

The robot follows a track using **three CNY70 infrared sensors** and
adjusts its movement based on the detected line position.

The system also supports **Bluetooth Low Energy (BLE)** communication
for remote control and telemetry.

------------------------------------------------------------------------

## 🧠 System Architecture

<p align="center">
  <img src="https://github.com/user-attachments/assets/25ffa88d-c747-4be2-97b5-714cad9d755d" width="700"/>
</p>

The ESP32 acts as the central controller that:

-   Reads the sensor values
-   Determines the required movement
-   Controls the motor drivers
-   Communicates via BLE

------------------------------------------------------------------------

## 🛠 Hardware Components

Main components used in the robot:

-   **ESP32 DevKit**
-   **3 × CNY70 infrared sensors**
-   **2 × L293D motor drivers**
-   **4 × DC gear motors**
-   **L7805CV voltage regulator**
-   Battery pack
-   Robot chassis

------------------------------------------------------------------------

## 🔌 Wiring Diagram

<p align="center">
  <img src="https://github.com/user-attachments/assets/32800341-a474-46e0-be57-80a89785e001" width="700"/>
</p>

The circuit includes:

-   ESP32 as the main controller
-   L293D drivers for motor control (using PWM)
-   CNY70 sensors for line detection
-   L7805CV voltage regulator for stable 5V power

------------------------------------------------------------------------

## ⚙ Control Logic

The robot operates using a simple line-following algorithm:

-   **Line centered** → move forward
-   **Line detected on left** → turn left
-   **Line detected on right** → turn right
-   **Line lost** → continue last correction

A third sensor detects **track markers** that control the robot speed.

------------------------------------------------------------------------

## 📡 BLE Control

The ESP32 exposes a **BLE service** for remote control.

### Commands

| Command | Function |
|--------|--------|
| D | Start driving |
| P | Stop driving |
| NORMAL | Normal speed mode |
| SPORT | High speed mode |
| ECHO | Stable slow mode |
### Telemetry format

Example:

    R=2800,L=2600,M=3000

Where:

-   **R** -- right sensor
-   **L** -- left sensor
-   **M** -- middle sensor

------------------------------------------------------------------------

## 💻 Running the Project

1.  Clone the repository
2.  Open the project in **Arduino IDE / PlatformIO**
3.  Upload the code to the **ESP32**
4.  Connect using a BLE app and send commands

------------------------------------------------------------------------

## 👨‍💻 Authors

Created by

**Dor Shitrit** and **Daniel Levi**
