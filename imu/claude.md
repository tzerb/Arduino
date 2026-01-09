# Claude Code Context

## Project Overview
Arduino R4 WiFi project reading sensor data from a GY-87 10-DOF IMU module.

## Hardware
- **Board**: Arduino R4 WiFi
- **IMU Module**: GY-87 (contains MPU6050 + QMC5883L + BMP180)

## Key Decisions Made
1. GY-87 module has QMC5883L magnetometer (Chinese clone), not HMC5883L
2. Using Adafruit and QMC5883LCompass libraries instead of raw I2C
3. BMP180 pressure reads ~2% low - within sensor tolerance
4. Old manual I2C implementation preserved as `oldSetup()`/`oldLoop()`

## Sensor Details
- **MPU6050**: Accelerometer (m/s²) + Gyroscope (rad/s) + Temperature
- **QMC5883L**: Magnetometer at I2C address 0x0D (accessed via MPU6050 bypass mode)
- **BMP180**: Barometer/temperature (only used in old implementation)

## Wiring
```
GY-87 VCC -> 3.3V
GY-87 GND -> GND
GY-87 SDA -> SDA (dedicated I2C pin, next to AREF)
GY-87 SCL -> SCL (dedicated I2C pin, next to SDA)
```

## Required Libraries
- Adafruit MPU6050
- Adafruit Unified Sensor
- QMC5883LCompass

## Notes
- Accelerometer measures gravity even when stationary (~9.8 m/s² distributed across axes based on tilt)
- MPU6050 I2C bypass must be enabled to access QMC5883L directly
- Altitude formula: `44330 * (1 - (P/P0)^0.1903)` where P0 = 101325 Pa
