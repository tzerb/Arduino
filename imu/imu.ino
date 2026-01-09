/*
 * GY-87 IMU Basic Example
 * Arduino R4 WiFi
 *
 * Reads data from:
 *   - MPU6050 (accelerometer/gyroscope)
 *   - QMC5883L (magnetometer)
 *   - BMP180 (barometer/temperature) - using MPU6050 library temperature
 *
 * Wiring:
 *   GY-87 VCC -> 3.3V
 *   GY-87 GND -> GND
 *   GY-87 SDA -> SDA (dedicated I2C pin, next to AREF)
 *   GY-87 SCL -> SCL (dedicated I2C pin, next to SDA)
 */

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <QMC5883LCompass.h>

Adafruit_MPU6050 mpu;
QMC5883LCompass compass;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("GY-87 IMU Initialization (Library Version)");
  Serial.println("==========================================");

  Wire.begin();

  // Initialize MPU6050
  if (mpu.begin()) {
    Serial.println("MPU6050: OK");
    // Enable I2C bypass to access QMC5883L
    mpu.setI2CBypass(true);
  } else {
    Serial.println("MPU6050: FAILED");
    while (1) delay(10);
  }

  // Initialize QMC5883L compass
  compass.init();
  Serial.println("QMC5883L: OK");

  Serial.println("\nStarting readings...\n");
  delay(1000);
}

void loop() {
  // Get MPU6050 sensor events
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Read compass
  compass.read();

  // Print accelerometer (m/s^2)
  Serial.print("Accel (m/s2): ");
  Serial.print(accel.acceleration.x, 2); Serial.print(", ");
  Serial.print(accel.acceleration.y, 2); Serial.print(", ");
  Serial.println(accel.acceleration.z, 2);

  // Print gyroscope (rad/s)
  Serial.print("Gyro (rad/s): ");
  Serial.print(gyro.gyro.x, 2); Serial.print(", ");
  Serial.print(gyro.gyro.y, 2); Serial.print(", ");
  Serial.println(gyro.gyro.z, 2);

  // Print magnetometer
  Serial.print("Mag (raw): ");
  Serial.print(compass.getX()); Serial.print(", ");
  Serial.print(compass.getY()); Serial.print(", ");
  Serial.println(compass.getZ());

  // Print compass azimuth
  Serial.print("Azimuth: ");
  Serial.print(compass.getAzimuth());
  Serial.println(" deg");

  // Print temperature from MPU6050
  Serial.print("Temp: ");
  Serial.print(temp.temperature, 1);
  Serial.println(" C");

  Serial.println("---");
  delay(500);
}

// ============================================================
// OLD MANUAL IMPLEMENTATION BELOW
// ============================================================

// I2C Addresses
#define MPU6050_ADDR    0x68
#define HMC5883L_ADDR   0x1E  // Same address for QMC5883L
#define QMC5883L_ADDR   0x0D  // Some QMC5883L use this address
#define BMP180_ADDR     0x77

// MPU6050 Registers
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_INT_PIN_CFG  0x37

// HMC5883L Registers
#define HMC5883L_CONFIG_A    0x00
#define HMC5883L_CONFIG_B    0x01
#define HMC5883L_MODE        0x02
#define HMC5883L_DATA_OUT    0x03

// QMC5883L Registers
#define QMC5883L_DATA_OUT    0x00
#define QMC5883L_STATUS      0x06
#define QMC5883L_CONFIG1     0x09
#define QMC5883L_CONFIG2     0x0A
#define QMC5883L_SET_RESET   0x0B

// BMP180 Registers
#define BMP180_CONTROL      0xF4
#define BMP180_RESULT       0xF6
#define BMP180_READ_TEMP    0x2E
#define BMP180_READ_PRESSURE 0x34

// Sensor data
int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;
int16_t magX, magY, magZ;
int32_t temperature, pressure;
float altitude;

// BMP180 calibration data
int16_t bmpAC1, bmpAC2, bmpAC3, bmpB1, bmpB2, bmpMB, bmpMC, bmpMD;
uint16_t bmpAC4, bmpAC5, bmpAC6;

// Magnetometer type (0 = HMC5883L, 1 = QMC5883L at 0x0D, 2 = QMC5883L at 0x1E)
uint8_t magType = 0;
uint8_t magAddr = HMC5883L_ADDR;

void oldSetup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("GY-87 IMU Initialization");
  Serial.println("========================");

  Wire.begin();
  delay(100);

  // Initialize MPU6050
  if (initMPU6050()) {
    Serial.println("MPU6050: OK");
  } else {
    Serial.println("MPU6050: FAILED");
  }

  // Initialize magnetometer (detect HMC5883L vs QMC5883L)
  if (initMagnetometer()) {
    if (magType == 0) {
      Serial.println("HMC5883L: OK");
    } else {
      Serial.print("QMC5883L: OK (addr 0x");
      Serial.print(magAddr, HEX);
      Serial.println(")");
    }
  } else {
    Serial.println("Magnetometer: FAILED");
  }

  // Initialize BMP180
  if (initBMP180()) {
    Serial.println("BMP180: OK");
  } else {
    Serial.println("BMP180: FAILED");
  }

  Serial.println("\nStarting readings...\n");
  delay(1000);
}

void oldLoop() {
  // Read all sensors
  readMPU6050();
  readMagnetometer();
  readBMP180();

  // Print accelerometer (g)
  Serial.print("Accel (g): ");
  Serial.print(accelX / 16384.0, 2); Serial.print(", ");
  Serial.print(accelY / 16384.0, 2); Serial.print(", ");
  Serial.println(accelZ / 16384.0, 2);

  // Print gyroscope (deg/s)
  Serial.print("Gyro (deg/s): ");
  Serial.print(gyroX / 131.0, 1); Serial.print(", ");
  Serial.print(gyroY / 131.0, 1); Serial.print(", ");
  Serial.println(gyroZ / 131.0, 1);

  // Print magnetometer (raw)
  Serial.print("Mag (raw): ");
  Serial.print(magX); Serial.print(", ");
  Serial.print(magY); Serial.print(", ");
  Serial.println(magZ);

  // Print temperature, pressure, and altitude
  Serial.print("Temp: ");
  Serial.print(temperature / 10.0, 1);
  Serial.print(" C  |  Pressure: ");
  Serial.print(pressure);
  Serial.print(" Pa  |  Alt: ");
  Serial.print(altitude, 1);
  Serial.println(" m");

  Serial.println("---");
  delay(500);
}

// ============ MPU6050 Functions ============

bool initMPU6050() {
  // Check if device is present
  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() != 0) return false;

  // Wake up MPU6050
  writeRegister(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
  delay(100);

  // Enable I2C bypass to access magnetometer directly
  writeRegister(MPU6050_ADDR, MPU6050_INT_PIN_CFG, 0x02);
  delay(10);

  return true;
}

void readMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14);

  // Read accelerometer
  accelX = (Wire.read() << 8) | Wire.read();
  accelY = (Wire.read() << 8) | Wire.read();
  accelZ = (Wire.read() << 8) | Wire.read();

  // Skip temperature (2 bytes)
  Wire.read(); Wire.read();

  // Read gyroscope
  gyroX = (Wire.read() << 8) | Wire.read();
  gyroY = (Wire.read() << 8) | Wire.read();
  gyroZ = (Wire.read() << 8) | Wire.read();
}

// ============ Magnetometer Functions ============

bool initMagnetometer() {
  // Try QMC5883L at 0x0D first (most common for clones)
  Wire.beginTransmission(QMC5883L_ADDR);
  if (Wire.endTransmission() == 0) {
    magType = 1;
    magAddr = QMC5883L_ADDR;
    return initQMC5883L(QMC5883L_ADDR);
  }

  // Try address 0x1E - could be HMC5883L or QMC5883L
  Wire.beginTransmission(HMC5883L_ADDR);
  if (Wire.endTransmission() == 0) {
    // Read chip ID register to distinguish
    // QMC5883L has chip ID 0xFF at register 0x0D
    uint8_t chipId = readRegister(HMC5883L_ADDR, 0x0D);
    if (chipId == 0xFF) {
      // It's a QMC5883L at 0x1E
      magType = 2;
      magAddr = HMC5883L_ADDR;
      return initQMC5883L(HMC5883L_ADDR);
    } else {
      // It's an HMC5883L
      magType = 0;
      magAddr = HMC5883L_ADDR;
      return initHMC5883L();
    }
  }

  return false;
}

bool initHMC5883L() {
  // Configure: 8 samples avg, 15Hz output, normal measurement
  writeRegister(HMC5883L_ADDR, HMC5883L_CONFIG_A, 0x70);
  // Set gain
  writeRegister(HMC5883L_ADDR, HMC5883L_CONFIG_B, 0x20);
  // Continuous measurement mode
  writeRegister(HMC5883L_ADDR, HMC5883L_MODE, 0x00);
  delay(10);
  return true;
}

bool initQMC5883L(uint8_t addr) {
  // Soft reset
  writeRegister(addr, QMC5883L_SET_RESET, 0x01);
  delay(10);
  // Configure: continuous mode, 200Hz, 8G range, 512 oversampling
  writeRegister(addr, QMC5883L_CONFIG1, 0x1D);
  // Enable interrupt pin and pointer rollover
  writeRegister(addr, QMC5883L_CONFIG2, 0x40);
  delay(10);
  return true;
}

void readMagnetometer() {
  if (magType == 0) {
    readHMC5883L();
  } else {
    readQMC5883L();
  }
}

void readHMC5883L() {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(HMC5883L_DATA_OUT);
  Wire.endTransmission(false);
  Wire.requestFrom(HMC5883L_ADDR, 6);

  // HMC5883L outputs X, Z, Y (not X, Y, Z)
  magX = (Wire.read() << 8) | Wire.read();
  magZ = (Wire.read() << 8) | Wire.read();
  magY = (Wire.read() << 8) | Wire.read();
}

void readQMC5883L() {
  Wire.beginTransmission(magAddr);
  Wire.write(QMC5883L_DATA_OUT);
  Wire.endTransmission(false);
  Wire.requestFrom(magAddr, 6);

  // QMC5883L outputs LSB first, X, Y, Z order
  magX = Wire.read() | (Wire.read() << 8);
  magY = Wire.read() | (Wire.read() << 8);
  magZ = Wire.read() | (Wire.read() << 8);
}

// ============ BMP180 Functions ============

bool initBMP180() {
  // Check if device is present
  Wire.beginTransmission(BMP180_ADDR);
  if (Wire.endTransmission() != 0) return false;

  // Read calibration data
  bmpAC1 = readInt16(BMP180_ADDR, 0xAA);
  bmpAC2 = readInt16(BMP180_ADDR, 0xAC);
  bmpAC3 = readInt16(BMP180_ADDR, 0xAE);
  bmpAC4 = readUInt16(BMP180_ADDR, 0xB0);
  bmpAC5 = readUInt16(BMP180_ADDR, 0xB2);
  bmpAC6 = readUInt16(BMP180_ADDR, 0xB4);
  bmpB1 = readInt16(BMP180_ADDR, 0xB6);
  bmpB2 = readInt16(BMP180_ADDR, 0xB8);
  bmpMB = readInt16(BMP180_ADDR, 0xBA);
  bmpMC = readInt16(BMP180_ADDR, 0xBC);
  bmpMD = readInt16(BMP180_ADDR, 0xBE);

  // Sanity check - AC1 should not be 0 or -1
  if (bmpAC1 == 0 || bmpAC1 == -1) return false;

  return true;
}

void readBMP180() {
  // Read raw temperature (16-bit)
  writeRegister(BMP180_ADDR, BMP180_CONTROL, BMP180_READ_TEMP);
  delay(5);
  int32_t UT = readInt16(BMP180_ADDR, BMP180_RESULT);

  // Read raw pressure (19-bit, need to read 3 bytes)
  writeRegister(BMP180_ADDR, BMP180_CONTROL, BMP180_READ_PRESSURE);
  delay(5);

  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(BMP180_RESULT);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP180_ADDR, (uint8_t)3);
  int32_t UP = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
  UP >>= 8;  // oss=0, shift by (8-oss)

  // Calculate true temperature (from datasheet)
  int32_t X1 = ((UT - (int32_t)bmpAC6) * (int32_t)bmpAC5) >> 15;
  int32_t X2 = ((int32_t)bmpMC << 11) / (X1 + (int32_t)bmpMD);
  int32_t B5 = X1 + X2;
  temperature = (B5 + 8) >> 4;  // Temperature in 0.1 C

  // Calculate true pressure (from datasheet, oss=0)
  int32_t B6 = B5 - 4000;
  X1 = ((int32_t)bmpB2 * ((B6 * B6) >> 12)) >> 11;
  X2 = ((int32_t)bmpAC2 * B6) >> 11;
  int32_t X3 = X1 + X2;
  int32_t B3 = (((int32_t)bmpAC1 * 4 + X3) + 2) >> 2;
  X1 = ((int32_t)bmpAC3 * B6) >> 13;
  X2 = ((int32_t)bmpB1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  uint32_t B4 = ((uint32_t)bmpAC4 * (uint32_t)(X3 + 32768)) >> 15;
  uint32_t B7 = (uint32_t)(UP - B3) * 50000UL;

  int32_t p;
  if (B7 < 0x80000000UL) {
    p = (B7 * 2) / B4;
  } else {
    p = (B7 / B4) * 2;
  }

  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;
  pressure = p + ((X1 + X2 + 3791) >> 4);  // Pressure in Pa

  // Calculate altitude using barometric formula
  // altitude = 44330 * (1 - (P/P0)^0.1903)
  // P0 = sea level pressure (101325 Pa)
  altitude = 44330.0 * (1.0 - pow((float)pressure / 101325.0, 0.1903));
}

// ============ I2C Helper Functions ============

void writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);
  return Wire.read();
}

int16_t readInt16(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)2);
  return (Wire.read() << 8) | Wire.read();
}

uint16_t readUInt16(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)2);
  return (Wire.read() << 8) | Wire.read();
}
