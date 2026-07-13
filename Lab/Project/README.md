# Motorcycle Security System

An Arduino-based smart security system for motorcycles with dual-stage motion detection, GPS tracking, GSM alerts, and geofencing capabilities.

## Features

### 🚨 Dual-Stage Motion Alarm
- **ADXL345 Accelerometer**: Detects motion in real-time
- **Stage 1 (Light Tap)**: Soft vibration + beep for < 3 seconds (threshold: 250mg)
- **Stage 2 (Sustained Motion)**: Loud siren with continuous vibration (threshold: 450mg)

### 📍 GPS Tracking & Geofencing
- Real-time GPS location tracking via TinyGPS++
- Geofence alerts when motorcycle moves beyond 10-meter radius
- GPS coordinates logged with tamper events

### 📱 GSM Alerts
- SMS notifications via A6 Pro Serial GPRS GSM Module
- Sends alerts to master phone number on security events
- Baud rate configurable (default 9600 bps)

### 🔌 Reed Switch Detection
- Detects motorcycle movement/vibration
- 500ms debounce to avoid false triggers
- Independent of motion alarm

### 🛡️ Anti-Tamper Protection
- **Lockout System**: Triggers after 3 alarms within 5 minutes to prevent false alarms
- **Secret Unlock Code**: 3-button sequence to disable lockout
- **Event Logging**: Records tamper attempts in EEPROM with timestamp, duration, and GPS location

### 🏍️ Ride Mode
- Disables motion detection during active riding (5-minute timer)
- Prevents false alarms while riding

### 🔑 Ignition Kill Switch
- Relay control to cut engine ignition when armed
- Prevents unauthorized motorcycle theft

### ⏰ Real-Time Clock (RTC)
- DS3231 RTC for accurate event timestamping
- Maintains time even during power loss

### 💡 LED Status Indicators
- RGB LEDs for system state visualization
- Binary digit display for status codes

### 🎮 Multi-Button Control
- Disarm button (D4)
- Mode button (D5)
- Secret button (D6)
- Key input sensor (D3)

## System States

| State | Behavior |
|-------|----------|
| **DISARMED** | Engine runnable, monitoring for auto-arm |
| **ARMED** | Monitoring for motion/tamper events |
| **WARNING** | Stage 1 alarm active (soft alert) |
| **ALARM** | Stage 2 alarm active (loud siren) |
| **LOCKOUT** | Triggered 3+ times in 5 minutes, ignores all sensors |

## Hardware Requirements

- Arduino Uno
- ADXL345 Accelerometer (I2C)
- DS3231 Real-Time Clock (I2C)
- A6 Pro Serial GPRS GSM Module
- GPS Module (software serial)
- Active Buzzer (PWM)
- Vibration Motor (PWM)
- Reed Switch
- Relay Module
- RGB LED
- Buttons & Key Sensor

## Pin Configuration

| Component | Pin | Type |
|-----------|-----|------|
| ADXL345 / RTC | A4 (SDA), A5 (SCL) | I2C |
| Buzzer | D9 | PWM |
| Vibration Motor | D10 | PWM |
| GSM Module | D7 (RX), D8 (TX) | Software Serial |
| GPS Module | A1 (RX), A2 (TX) | Software Serial |
| Buttons | D4, D5, D6 | Digital Input |
| Key Input | D3 | Digital Input |
| Reed Switch | D2 | Digital Input |
| Ignition Kill | A0 | Digital Output |

## Usage

1. Upload `MotorcycleSecuritySystem.ino` to Arduino Uno
2. Update `MASTER_PHONE` constant with your phone number (include country code)
3. Use `GSM_Module_BaudRate_Set.ino` to configure GSM module baud rate (if needed)
4. Arm the system using button controls
5. System will send SMS alerts to master phone on tamper detection

## Configuration

- **Stage 1 Threshold**: 250mg (light tap)
- **Stage 2 Threshold**: 450mg (sustained motion)
- **Geofence Radius**: 10 meters
- **Lockout Trigger**: 3 alarms within 5 minutes
- **SMS Check Interval**: 15 seconds
- **Geofence Check Interval**: 30 seconds

**If you experience issues with SMS alerts, try using MotorcycleSecuritySystem(without delay).ino.**
