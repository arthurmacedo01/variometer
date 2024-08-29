# Altitude Monitoring and Alert System using ESP32 and BMX280 Sensor

This project implements an altitude monitoring and alert system using an ESP32 microcontroller, a BMX280 sensor (for pressure and temperature measurements), and a buzzer for auditory alerts. The system utilizes a Kalman Filter to estimate altitude, velocity, and acceleration from sensor readings and provides real-time feedback through auditory cues.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Circuit Diagram](#circuit-diagram)
- [Setup and Installation](#setup-and-installation)
- [Kalman Filter Explanation](#kalman-filter-explanation)
- [Usage](#usage)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgements](#acknowledgements)

---

## Overview

The **Altitude Monitoring and Alert System** is designed to provide precise altitude measurements and corresponding alerts based on vertical velocity. It is ideal for applications such as drone flight control, hiking altimeters, or any system requiring accurate altitude and movement tracking.

The system reads pressure data from the BMX280 sensor, converts it to altitude, and processes it through a Kalman Filter to obtain smooth and accurate estimates of altitude, velocity, and acceleration. Based on the estimated velocity, the buzzer produces sound signals with varying frequencies, providing immediate auditory feedback on the vertical movement.

---

## Features

- **Accurate Altitude Measurement**: Utilizes the BMX280 sensor for precise pressure and temperature readings.
- **Kalman Filter Implementation**: Provides smooth and reliable estimates of altitude, velocity, and acceleration.
- **Real-time Auditory Alerts**: Buzzer frequency changes according to the vertical velocity, indicating ascent or descent.
- **Configurable Parameters**: Sampling time, noise covariances, and sensitivity can be adjusted as needed.
- **Modular Code Structure**: Easy to understand and modify for custom applications.
- **FreeRTOS Support**: Tasks are managed efficiently using FreeRTOS, ensuring real-time performance.

---

## Hardware Requirements

- **ESP32 Development Board**: Any standard ESP32 board will suffice.
- **BMX280 Sensor Module**: For measuring pressure and temperature.
- **Buzzer**: A piezoelectric buzzer for auditory alerts.
- **Connecting Wires**: For establishing connections between components.
- **Breadboard or PCB**: Optional, for organizing and securing connections.
- **Power Supply**: USB cable or appropriate power source for the ESP32 board.

### Pin Connections

| ESP32 GPIO Pin | Component     | Component Pin |
|----------------|---------------|---------------|
| GPIO 21        | BMX280 Sensor | SDA           |
| GPIO 22        | BMX280 Sensor | SCL           |
| GPIO 25        | Buzzer        | Signal (I/O)  |
| GND            | BMX280 Sensor | GND           |
| 3.3V           | BMX280 Sensor | VCC           |
| 3.3V           | Buzzer        | Power (+)     |
| GND            | Buzzer        | Ground (-)    |

---

## Software Requirements

- **ESP-IDF (Espressif IoT Development Framework)**: Version 4.x or later.
- **Python 3.x**: For ESP-IDF and related tools.
- **Git**: For version control and code management.
- **Serial Monitor**: Such as `minicom`, `putty`, or the built-in ESP-IDF monitor tool.

### Libraries and Dependencies

- **FreeRTOS**: Included in ESP-IDF.
- **LEDC Driver**: For PWM control of the buzzer (included in ESP-IDF).
- **I2C Driver**: For communication with the BMX280 sensor (included in ESP-IDF).
- **BMX280 Driver**: Ensure you have the appropriate driver/library for the BMX280 sensor compatible with ESP-IDF.

---

## Circuit Diagram

Below is a simplified circuit diagram illustrating the connections between the ESP32, BMX280 sensor, and the buzzer.

       +-----------------------+
       |        ESP32          |
       |                       |
       |   GPIO 25 (BUZZER) ---+-----+
       |   GPIO 22 (SCL) ------+---+ | 
       |   GPIO 21 (SDA) ------+-+ | | 
       |                       | | | |
       +-----------------------+ | | |
                                 | | |
      +----------------+         | | |
      |   BMX280       |         | | |
      |                |         | | |
      |   SDA ---------+---------+ | |
      |   SCL ---------------------+ |
      |   VCC ---------3.3V          |
      |   GND ---------GND           |
      +----------------+             |
                                     |
      +--------+                     |
      | Buzzer |                     |
      |        |                     |
      |  + ----+--- 3.3V             |
      |  - ----+--- GND              |   
      |  I/O --+--- GPIO 25 ---------+
      +--------+


**Note**: For better performance and safety, consider adding pull-up resistors (typically 4.7kΩ) on the SDA and SCL lines if they are not included on your sensor module.

---

## Setup and Installation

### 1. Install ESP-IDF

Follow the official [ESP-IDF Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html) to set up the development environment on your operating system.

### 2. Clone the Repository

```bash
git clone https://github.com/arthurmacedo01/variometer.git
cd variometer
```

### 3. Configure the Project

Set up the project configuration using menuconfig:

```bash
idf.py menuconfig
```

Ensure all settings are correct, especially the serial port and other hardware-specific configurations.
### 4. Build the Project

Compile the project using:

```bash
idf.py build
```

### 5. Flash the Firmware

Connect your ESP32 board to the computer and flash the compiled firmware:

```bash
idf.py flash
```

### 6. Monitor the Output

After flashing, you can monitor the serial output:

```bash
idf.py monitor
```

Press Ctrl+] to exit the monitor.

### Kalman Filter Explanation

The Kalman Filter is an efficient recursive filter that estimates the state of a dynamic system from a series of incomplete and noisy measurements. In this project, it is used to estimate:

    Altitude (x1)
    Vertical Velocity (x2)
    Vertical Acceleration (x3)

## Kalman Filter Steps

    Prediction Step:
        Predicts the next state (x_hat_pred) based on the previous state and a dynamic model.
        Predicts the error covariance (P_pred) of the estimate.

    Update Step:
        Incorporates new measurements to correct the predicted state.
        Calculates the Kalman Gain (K) which determines the weight given to the new measurements.
        Updates the state estimate (x_hat) and error covariance (P).

## Parameters

    Sampling Time (T): Time interval between measurements (100 ms).
    Process Noise Covariance (Q1, Q2, Q3): Represents the uncertainty in the model.
    Measurement Noise Covariance (R): Represents the uncertainty in the measurements.
    Initial Error Covariance (P11, P12, ..., P33): Represents the initial uncertainty in the state estimates.

## Benefits

    Noise Reduction: Provides smooth estimates by filtering out sensor noise.
    Predictive Capability: Can predict future states, useful for proactive alerting.
    Adaptive: Automatically adjusts estimates based on measurement accuracy.

### Usage

Once the system is set up and running:

    Power On: Connect and power up the ESP32 board.
    Monitoring: Use the serial monitor to observe real-time data, including current pressure, altitude, and estimated states.
    Auditory Alerts:
        Ascending: The buzzer emits higher-frequency sounds when ascending.
        Descending: The buzzer emits lower-frequency sounds when descending.
        Stationary: The buzzer remains silent or emits a baseline frequency within the defined dead band.

## Configurable Parameters

    DEAD_BAND: Threshold for minimal velocity below which the buzzer remains silent.
    SENSIBILITY: Factor determining how much the buzzer frequency changes with velocity.
    Sampling Time (T): Adjust for faster or slower
