# Adaptive Suspension System

## Overview
- Arduino-based suspension system that adjusts **height** and **stiffness** automatically or manually.
- Uses **pneumatic actuators**, **motorized bypass shocks**, and **airbag-assisted coilovers**.
- Reads vehicle tilt angle to prevent rollovers on steep inclines.
- Can be controlled manually with buttons and switches.

## Features
- **3 Suspension Modes**: Soft, Normal, Stiff
- **Auto Height Adjustment** based on gyro sensor
- **Manual Height Control** using a 3-position switch
- **LED Indicators** for system status
- **Fail-safe System** to prevent incorrect adjustments

## Hardware Used
- **Arduino Mega** – Main controller
- **Pneumatic Actuators** – Adjusts vehicle height
- **Airbag-Assisted Coilovers** – Modifies suspension stiffness
- **Motorized Bypass Shocks** – Controls damping force
- **Pressure Sensors** – Reads air pressure in suspension
- **Gyroscope & Accelerometer** – Detects vehicle tilt
- **3-Position Switch** – Manual height control (Up/Down/Neutral)
- **Push Buttons** – Select suspension modes
- **LED Indicators** – Shows system status

## Installation

### 1. Clone Repository
```bash
git clone https://github.com/nixonsardjono/Adaptive-Suspension-System.git
cd Adaptive-Suspension-System
