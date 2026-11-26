# IoT-Based Smart Industrial Machine Health Monitoring and Early Fault Detection System

## Project Overview

An ESP32-based IoT industrial machine monitoring prototype designed to monitor machine operating conditions, detect abnormal behaviour and provide early fault alerts.

The system integrates multiple sensors to monitor vibration, acceleration, motor current, ambient temperature and humidity. Sensor readings are processed by the ESP32 and transmitted using MQTT, while Blynk provides real-time remote monitoring.

The project also aims to implement automatic fault diagnosis, controlled motor shutdown and recovery verification.

## Hardware Components

| Component | Purpose |
|---|---|
| ESP32 DevKit V1 | Main controller |
| MPU6050 | Acceleration and vibration monitoring |
| SW-420 | Vibration event detection |
| ACS712 20A | Motor current measurement |
| DHT11 | Ambient temperature and humidity |
| L298N | DC motor control |
| DC Motor | Machine test load |
| OLED 0.96-inch | Local monitoring |
| LEDs | Machine status indication |
| Buzzer | Fault alarm |
| 3S Li-ion battery | Motor power supply |

## Software and Technologies

- Embedded C/C++
- Arduino IDE
- ESP32
- Blynk IoT
- MQTT
- EMQX MQTT Broker

## System Architecture

1. Sensors collect machine operating parameters.
2. ESP32 reads and processes sensor data.
3. Fault conditions are evaluated using programmed thresholds.
4. Machine telemetry is published through MQTT.
5. Blynk displays real-time monitoring information.
6. Local OLED, LEDs and buzzer provide alerts.
7. Planned protection logic supports controlled motor shutdown and recovery verification.

## Key Features

- Real-time machine health monitoring
- Vibration and acceleration monitoring
- Motor current monitoring
- Ambient temperature and humidity monitoring
- MQTT-based telemetry
- Blynk IoT dashboard
- Fault detection and diagnosis
- Planned automatic motor protection and recovery verification

## Project Status

**Prototype development and testing in progress.**

Verified components and functions include sensor communication, motor operation, OLED monitoring, Blynk telemetry and MQTT telemetry reception.

Automatic shutdown and recovery features require further implementation and validation.

## Future Improvements

- Motor surface temperature monitoring using DS18B20
- DC-rated contactor-based motor isolation
- Advanced fault diagnosis
- Automatic recovery verification
- Historical machine health analysis
- AI/ML-based predictive maintenance

## Applications

- Industrial motor monitoring
- Machine condition monitoring
- Industrial IoT prototypes
- Early fault detection systems
- Predictive maintenance research

## Author

Embedded Systems and IoT Project

## License

MIT License (if selected for this repository).
