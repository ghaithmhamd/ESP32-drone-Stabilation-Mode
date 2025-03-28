# ESP32-Drone-Stabilation-Mode-V1.0

## Overview
This project implements **Angle Mode + Rate Mode** using the IMU's (**MPU6050**) Gyroscope and Accelerometer for drone stabilization. The code is available on the **postery**, and a demonstration video is available on Facebook: [Video Link](https://www.facebook.com/share/p/153RfBXNjC/).

## Features
- **Angle Mode**: Uses accelerometer and gyroscope data for stable flight.
- **Rate Mode**: Uses gyroscope data only for acrobatic maneuvers.
- **PID Control**: Implements a **PID loop** for precise stabilization.
- **ESP32-S3-Zero**: Handles sensor fusion and motor control.

## Transmitter
The transmitter code is available in the following repository: [Radio Transmitter and Receiver](https://github.com/ghaithmhamd/Radio-transmitter-and-reciever)

## Hardware Requirements
- **ESP32-S3-Zero** (Main flight controller)
- **MPU6050** (IMU sensor)
- **NRF24L01** (Wireless communication module)
- **Brushless Motors & ESCs**
- **Battery & Power Distribution Board**

## Installation & Setup
1. Clone the repository.
2. Upload the code to your **ESP32-S3-Zero** using **Arduino IDE**.
3. Ensure proper sensor calibration before flight.
4. Connect the transmitter (linked above) and test the response.

## Notes
- Ensure that the **IMU is properly mounted** to avoid drift.
- The **PID parameters** might need tuning based on your drone's configuration.

## License
This project is open-source. Feel free to contribute and improve it! ✈️

