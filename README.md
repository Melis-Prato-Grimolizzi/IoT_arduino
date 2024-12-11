# LoRa Parking Sensor System

## Overview

This project implements a parking sensor system using ultrasonic sensors and a LoRa module for communication. It monitors parking slots to determine if they are occupied or free, and sends status updates via the LoRa communication protocol. Each parking slot is equipped with two ultrasonic sensors for reliable detection.

## Features

- **Ultrasonic Sensors**: Utilizes the NewPing library for precise distance measurements.
- **LoRa Communication**: Sends state updates of parking slots to a receiver.
- **Stability Logic**: Ensures stable detection and avoids false positives.
- **LED Indicators**: Visual feedback for the state of each parking slot (occupied or free).

## Hardware Requirements

- **Arduino Uno** or a compatible microcontroller.
- **E220-900T22D LoRa Module**.
- **HC-SR04 Ultrasonic Sensors** (or equivalent).
- LEDs for state indication (one per parking slot).
- Jumper wires and a breadboard for connections.

## Pin Configuration

### Ultrasonic Sensors

| Slot | Trigger Pin | Echo Pin |
| ---- | ----------- | -------- |
| 1    | 13          | 12       |
| 2    | 11          | 10       |
| 3    | 9           | 8        |

### LoRa Module

| Pin | Function |
| --- | -------- |
| 2   | RXD      |
| 3   | TXD      |

### LED Indicators

| Slot | LED Pin |
| ---- | ------- |
| 1    | 7       |
| 2    | 6       |
| 3    | 5       |

## Software Dependencies

- **Arduino IDE**
- **NewPing Library**: Optimized library for ultrasonic sensors.
- **LoRa\_E220 Library**: For interfacing with the E220-900T22D LoRa module.

## Code Structure

### Key Components

1. **`Sensor`**: Represents an ultrasonic sensor with its state (ON/OFF).
2. **`Slot`**: Represents a parking slot with associated sensors and state.
3. **`packet`**: Structure for communication packets.
4. **`timeout()`**: Utility function to manage timing logic.

### Workflow

1. Initialize sensors, LoRa module, and LEDs.
2. Continuously monitor sensors in the `loop()` function.
3. Determine slot states based on sensor readings.
4. Transmit slot state changes via LoRa.
5. Update LED indicators to reflect the slot states.

## Installation

1. Clone or download this repository.
2. Install the required libraries in the Arduino IDE:
   - `NewPing`
   - `LoRa_E220`
3. Connect the hardware components as per the pin configuration.
4. Upload the `LoRa Parking Sensor System` code to the Arduino.

## Usage

- Upon powering the system, the LoRa module will initialize.
- The system continuously monitors the parking slots and determines their occupancy state.
- State changes (e.g., from free to occupied) are transmitted via LoRa and logged to the Serial Monitor.
- LEDs indicate the current state of each slot:
  - **LOW**: Slot is free.
  - **HIGH**: Slot is occupied.
