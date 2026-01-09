# GY-87 IMU Project - Arduino R4 WiFi

Read sensor data from a GY-87 10-DOF IMU module using an Arduino R4 WiFi.

## Hardware

### GY-87 Module Components

The GY-87 is a multi-sensor module containing:

| Sensor | Function | I2C Address | Data Provided |
|--------|----------|-------------|---------------|
| MPU6050 | Accelerometer + Gyroscope | 0x68 | Motion, orientation, temperature |
| QMC5883L | Magnetometer (compass) | 0x0D | Heading/azimuth |
| BMP180 | Barometer | 0x77 | Pressure, altitude, temperature |

**Note**: Many GY-87 modules ship with a QMC5883L (Chinese clone) instead of the original HMC5883L. The QMC5883L uses address 0x0D rather than 0x1E.

### Sensor Relationships

```
                    ┌─────────────────────────────────────┐
                    │           GY-87 Module              │
                    │                                     │
    I2C Bus ────────┼──┬─────────────────────────────────┤
                    │  │                                  │
                    │  ▼                                  │
                    │ ┌─────────┐    I2C Bypass          │
                    │ │ MPU6050 │◄──────────────┐        │
                    │ │ (0x68)  │               │        │
                    │ └────┬────┘               │        │
                    │      │                    │        │
                    │      │ Accel (X,Y,Z)      │        │
                    │      │ Gyro (X,Y,Z)       │        │
                    │      │ Temperature        │        │
                    │      │                    │        │
                    │      │              ┌─────┴─────┐  │
                    │      │              │ QMC5883L  │  │
                    │      │              │  (0x0D)   │  │
                    │      │              └─────┬─────┘  │
                    │      │                    │        │
                    │      │              Mag (X,Y,Z)    │
                    │      │              Azimuth        │
                    │      │                             │
                    │ ┌────┴────┐                        │
                    │ │ BMP180  │                        │
                    │ │ (0x77)  │                        │
                    │ └────┬────┘                        │
                    │      │                             │
                    │      │ Pressure                    │
                    │      │ Altitude                    │
                    │      │ Temperature                 │
                    │                                    │
                    └────────────────────────────────────┘
```

**Important**: The QMC5883L is connected behind the MPU6050. You must enable I2C bypass mode on the MPU6050 (`mpu.setI2CBypass(true)`) before the magnetometer becomes visible on the I2C bus.

## Wiring

| GY-87 Pin | Arduino R4 WiFi |
|-----------|-----------------|
| VCC | 3.3V |
| GND | GND |
| SDA | SDA (dedicated I2C pin, next to AREF) |
| SCL | SCL (dedicated I2C pin, next to SDA) |

## Required Libraries

Install via Arduino IDE Library Manager:

1. **Adafruit MPU6050** - Accelerometer/gyroscope driver
2. **Adafruit Unified Sensor** - Required dependency
3. **QMC5883LCompass** - Magnetometer driver (by MPrograms)

## Sample Output

```
GY-87 IMU Initialization (Library Version)
==========================================
MPU6050: OK
QMC5883L: OK

Starting readings...

Accel (m/s2): 0.47, 2.34, 9.96
Gyro (rad/s): -0.06, 0.01, 0.01
Mag (raw): -748, -560, -2082
Azimuth: -143 deg
Temp: 27.7 C
---
```

## Understanding the Data

### Accelerometer (m/s²)
Measures acceleration including gravity. A stationary board shows ~9.8 m/s² distributed across axes based on orientation:
- Flat board: X≈0, Y≈0, Z≈9.8
- Tilted board: Gravity vector splits across multiple axes

To calculate tilt angles:
- Roll = atan2(Y, Z)
- Pitch = atan2(-X, sqrt(Y² + Z²))

### Gyroscope (rad/s)
Measures rotational velocity around each axis. Values near zero when stationary.

### Magnetometer (raw)
Raw magnetic field readings. The library calculates azimuth (compass heading) from these values.

### Azimuth (degrees)
Compass heading: 0° = North, 90° = East, 180° = South, 270° = West

## Code Structure

The sketch contains two implementations:

1. **Library-based** (active): Uses Adafruit and QMC5883LCompass libraries
   - `setup()` / `loop()`

2. **Manual I2C** (preserved): Direct register access, includes BMP180 support
   - `oldSetup()` / `oldLoop()`
   - Includes pressure, altitude calculation
   - Auto-detects HMC5883L vs QMC5883L

To use the old implementation, rename the functions in the sketch.

## Future Enhancements

- Sensor fusion (calculate roll/pitch/yaw)
- WiFi data streaming
- Calibration routines for magnetometer
- Add BMP180 library support to new implementation
