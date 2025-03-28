# ESP32-drone-Stabilation-Mode-V1.0

## Overview
This project implements **Angle Mode** and **Rate Mode** for an ESP32-based drone using an **MPU6050 IMU** (Gyroscope & Accelerometer). The stabilization algorithm ensures smooth and controlled flight dynamics.

## Features
✅ **Angle Mode** (Self-Leveling) - Uses the accelerometer for automatic leveling.
✅ **Rate Mode** (Acro Mode) - Uses only the gyroscope for direct control.
✅ **PID Control** - Fine-tuned stabilization system.
✅ **ESP32 Integration** - Lightweight and optimized for real-time control.

## Components Used
- **ESP32-S3-Zero** (Main Flight Controller)
- **MPU6050** (IMU Sensor for attitude estimation)
- **NRF24L01** (Wireless communication)
- **Brushless Motors + ESCs**
- **LiPo Battery (3S/4S)**

## Installation & Usage
1. **Flash the Code**: The full source code is available on the **Postery** platform.
2. **Connect the Hardware**: Ensure proper wiring between ESP32, MPU6050, and motors.
3. **Tune PID Parameters**: Adjust `Kp, Ki, Kd` for best stability.
4. **Test Flight**: Gradually increase throttle and observe stability.

## Demo Video 🎥
Watch the flight test here: [Facebook Video](https://www.facebook.com/share/p/153RfBXNjC/)

## Future Improvements
🔹 Altitude Hold Mode (using **BMP280**)
🔹 GPS Waypoint Navigation
🔹 Optical Flow for Indoor Stability

---
📌 **Contributions & Feedback** are welcome! Let’s make open-source drone control even better. 🚀

