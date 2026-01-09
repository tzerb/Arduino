# Arduino Projects

## Current Project: BluetoothSimpleService

A Bluetooth-controlled servo with LED matrix display for Arduino R4 WiFi.

### Features
- BLE service for remote servo control
- Servo position controlled via Bluetooth (values 0-9, multiplied by 20 for angle)
- LED matrix displays current servo value
- Serial monitor output for debugging
- Startup sequence with LED flash and servo sweep running simultaneously using millis()

### Hardware
- Arduino R4 WiFi
- Servo on pin 9
- Built-in 12x8 LED matrix
- Built-in LED (LED_BUILTIN)

### BLE Characteristics
- Service UUID: `19B10000-E8F2-537E-4F6C-D104768A1214`
- Servo Characteristic UUID: `19B10002-E8F2-537E-4F6C-D104768A1214`

### Notes
- R4 WiFi does NOT have RGB LED pins (LEDR, LEDG, LEDB) despite some documentation
- TX_LED and RX_LED did not work as expected
- Green power LED is hardwired and cannot be controlled via software
- No built-in motion sensor on R4 WiFi

### Libraries Used
- ArduinoBLE
- Servo
- ArduinoGraphics
- Arduino_LED_Matrix
- WiFiS3
